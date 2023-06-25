# #!/usr/bin/env python
import zmq
import time
import rospy
import threading
import time
import base64

from ik_service_custom import calculate_coordinate
from x_display_custom import send_image
from std_msgs.msg import (
    UInt16,
)

import baxter_interface
from baxter_interface import CHECK_VERSION

from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import json
import os

from sensor_msgs.msg import Image, CameraInfo
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetLinkProperties, GetLinkProperties
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion
from gazebo_msgs.msg import ModelState, ModelStates


class BaxterServer:
    def __init__(self):
        print("Initializing node... ")
        rospy.init_node("streamer_node")
        # pub_rate = rospy.Publisher(robot_ns + '/joint_state_publish_rate', UInt16, queue_size=10)
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 100Hz
		# pub_rate.publish(100)
        
        # init - need to add all the variables that can be controlled!
		# the speed is set to 1,0 (max) by default - this should be controlled by the user!
        self.arm_left = baxter_interface.Limb('left')                        
        self.arm_left.set_joint_position_speed(1.0)

        self.arm_right = baxter_interface.Limb('right')
        self.arm_right.set_joint_position_speed(1.0)
		
        self.gripper_left = baxter_interface.Gripper('left')
        self.gripper_right = baxter_interface.Gripper('right')
		
        self.head = baxter_interface.Head()
        self.bridge = CvBridge()
        head_image_topic = 'cameras/head_camera/image/'
        rospy.Subscriber(head_image_topic, Image, self.head_image_callback)
        left_image_topic = 'cameras/left_hand_camera/image/'
        rospy.Subscriber(left_image_topic, Image, self.left_image_callback)
        rigth_image_topic = 'cameras/right_hand_camera/image/'
        rospy.Subscriber(rigth_image_topic, Image, self.right_image_callback)
        # topic =  '/gazebo/model_states'
        # rospy.Subscriber(topic, ModelStates, self.gazebo_models_callback)
        # self.model_keys=[]

        #self.head_camera = baxter_interface.CameraController()
        self.image_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.init_stream()
        
    def init_stream(self):
        self.port = "15557"
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:%s" % self.port)

        self.port2 = "15558"
        self.socket2 = self.context.socket(zmq.REP)
        self.socket2.bind("tcp://*:%s" % self.port2)

        self.port3 = "15559"
        self.socket3 = self.context.socket(zmq.PUB)
        self.socket3.bind("tcp://*:%s" % self.port3)

        

        # Sleep a bit of time until the connection is properly established,
        # otherwise some messages may be lost.
        # http://stackoverflow.com/questions/7470472/lost-messages-on-zeromq-pub-sub
        time.sleep(0.5)

        print("Publisher initialized.")


    def set_neutral(self):
        print("Moving to neutral pose...")
        self.arm_left.move_to_neutral()
        self.arm_right.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
                
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True
    
    def head_image_callback(self, msg):
        self.head_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def left_image_callback(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # cv2.imwrite("camera.jpeg",self.left_img)

    def right_image_callback(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def publish_data(self):
        
        while not rospy.is_shutdown():
            self.data_lock.acquire()
            data = {
                'arm_left': {
                                'angles': self.arm_left.joint_angles(),
                                'velocities': self.arm_left.joint_velocities()
                            },
                'arm_right': {
                                'angles': self.arm_right.joint_angles(),
                                'velocities': self.arm_right.joint_velocities()
                            },
                'left_endpoint_position':{ 
                                'data':self.arm_left.endpoint_pose()

                },
                'right_endpoint_position':{ 
                                'data':self.arm_right.endpoint_pose()

                },
                'gripper_right': { },
					                    
                #'head_img': json.dumps(self.head_img.tolist()),
                #'left_img': json.dumps(self.left_img.tolist()),
                #'right_img': json.dumps(self.right_img.tolist()),
            }
            self.socket.send_json(data)
            time.sleep(0.2)
            self.data_lock.release()
            
    
    def publish_image(self):

        while not rospy.is_shutdown():

            # strng = base64.b64encode(self.left_img).decode("utf-8")
            self.image_lock.acquire()
            cv2.imwrite("camera.jpeg",self.right_img)
            filename = "camera.jpeg"

            # right_image = self.right_img
            # TODO: complete mutexing
            
            with open(filename,"r") as image_file:
                encoded_string = base64.b64encode(image_file.read())

            # encoded_string = base64.b64encode(right_image)
            image_file.close()
            # data = {
					                    
            #     #'head_img': json.dumps(self.head_img.tolist()),
            #     'left_img': json.dumps(strng),
            #     #'right_img': json.dumps(self.right_img.tolist()),
            # }
            self.socket3.send_string(encoded_string)
            time.sleep(0.5)
            self.image_lock.release()

    def publish_image2(self):
        self.image_lock.acquire()
        cv2.imwrite("camera.jpeg",self.right_img)
        filename = "camera.jpeg"
        with open(filename,"r") as image_file:
            encoded_string = base64.b64encode(image_file.read())
        image_file.close()
        self.socket2.send_string(encoded_string)
        time.sleep(0.5)
        self.image_lock.release()

    def publish_image3(self):
        self.image_lock.acquire()
        cv2.imwrite("camera.jpeg",self.left_img)
        filename = "camera.jpeg"
        with open(filename,"r") as image_file:
            encoded_string = base64.b64encode(image_file.read())
        image_file.close()
        self.socket2.send_string(encoded_string)
        time.sleep(0.5)
        self.image_lock.release()

    def get_angles(self):
        data = {
                'arm_left': {
                                'angles': self.arm_left.joint_angles(),
                                'velocities': self.arm_left.joint_velocities()
                            },
                'arm_right': {
                                'angles': self.arm_right.joint_angles(),
                                'velocities': self.arm_right.joint_velocities()
                            },
                'left_endpoint_position':{ 
                                'data':self.arm_left.endpoint_pose()

                },
                'right_endpoint_position':{ 
                                'data':self.arm_right.endpoint_pose()

                },
                'gripper_right': { }
            }
        self.socket2.send_json(data)

    def run(self):
        """
        """
        rate = rospy.Rate(10)

        print("Publishing robot state ...")

        self.gripper_left.open()
        self.gripper_right.open()

        t1 = threading.Thread(target=self.publish_data)
        t1.start()

        # while not rospy.is_shutdown():   
        #     data = {
        #         'arm_left': {
        #                         'angles': self.arm_left.joint_angles(),
        #                         'velocities': self.arm_left.joint_velocities()
        #                     },
        #         'arm_right': {
        #                         'angles': self.arm_right.joint_angles(),
        #                         'velocities': self.arm_right.joint_velocities()
        #                     },    
        #         'gripper_right': { },
					                    
        #         #'head_img': json.dumps(self.head_img.tolist()),
        #         #'left_img': json.dumps(self.left_img.tolist()),
        #         #'right_img': json.dumps(self.right_img.tolist()),
        #     }
                    
            #self.socket.send_json(data)	
            
            # Check commands
        while not rospy.is_shutdown():

            msg = self.socket2.recv_json()
            print(msg) 
                
            if 'head' in msg:
                par = msg['head']
                self.head.command_nod()
                print("Head nodded!")
            
            if 'smiley' in msg:
                send_image("smiley.png")
            
            if 'kinectAngle' in msg:
                print("megvan")
                angle = msg['kinectAngle']
                print(angle)
                self.socket2.send("recived")
            image_sent = False
            if 'left_arm' in msg:
                par = msg['left_arm']
                if 'angles' in par:
                    self.get_angles()
                if 'move_to' in par:
                    angles = par['move_to']
                    self.arm_left.set_joint_position_speed(1)
                    self.arm_left.move_to_joint_positions(angles)
                    print("Arm moved to positions")
                    self.socket2.send("Arm moved to position")
                if 'set_joint_positions' in par:
                    angles = par['set_joint_positions']
                    self.arm_left.set_joint_position_speed(0.7)
                    self.arm_left.set_joint_positions(angles)
                    print("Angles set to positions")
                    self.socket2.send("Angles set to positions")
                if 'gripper' in par:
                    if par['gripper'] == 1:
                        self.gripper_left.open()
                        # self.gripper_right.open()
                        print("Grip opened")
                        self.socket2.send("Grip opened!")
                    else:
                        self.gripper_left.close()
                        # self.gripper_right.close()
                        print("Grip closed")
                        self.socket2.send("Grip Closed")
                if 'ball_coordinate' in par:
                    coordinates = par['ball_coordinate']
                    ball_angels = calculate_coordinate('left',coordinates['x'],coordinates['y'],coordinates['z'],coordinates['ox'],coordinates['oy'],coordinates['oz'],coordinates['ow'])
                    self.arm_left.move_to_joint_positions(ball_angels)
                    print("Arm moved to positions")
                    self.socket2.send("Moved to position")
                if 'image' in par:
                    if par['image'] == 1:
                        self.publish_image3()
                        image_sent = True
            if 'right_arm' in msg:
                par = msg['right_arm']
                if 'angles' in par:
                    self.get_angles() 
                if 'move_to' in par:
                    angles = par['move_to']
                    self.arm_right.set_joint_position_speed(1)
                    self.arm_right.move_to_joint_positions(angles)
                    print("Arm moved to positions")
                    self.socket2.send("Arm moved to position")
                if 'set_joint_positions' in par:
                    angles = par['set_joint_positions']
                    self.arm_right.set_joint_position_speed(0.7)
                    self.arm_right.set_joint_positions(angles)
                    print("Angles set to positions")
                    self.socket2.send("Angles set to positions")
                if 'gripper' in par:
                    if par['gripper'] == 1:
                        self.gripper_right.open()
                        # self.gripper_right.open()
                        print("Grip opened")
                        self.socket2.send("Grip opened!")
                    else:
                        self.gripper_right.close()
                        # self.gripper_right.close()
                        print("Grip closed")
                        self.socket2.send("Grip Closed")
                if 'image' in par:
                    if par['image'] == 1:
                        self.publish_image2()
                        image_sent = True
                if 'ball_coordinate' in par:
                    coordinates = par['ball_coordinate']
                    ball_angels = calculate_coordinate('right',coordinates['x'],coordinates['y'],coordinates['z'],coordinates['ox'],coordinates['oy'],coordinates['oz'],coordinates['ow'])
                    self.arm_right.move_to_joint_positions(ball_angels)
                    print("Arm moved to positions")
                    self.socket2.send("Moved to position")
                # if 'image' in par: 
                #     self.publish_image()

            # self.socket2.send("Command received!")
            cmd = {}
                
            rate.sleep()

def main():
    server = BaxterServer()
    rospy.on_shutdown(server.clean_shutdown)
    server.run()

    print("Done.")
    return 0

if __name__ == '__main__':
    main()
