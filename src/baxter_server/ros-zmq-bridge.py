# #!/usr/bin/env python
import zmq
import time
import rospy
import threading
import time
import base64

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
            }
            self.socket.send_json(data)
            time.sleep(0.2)
            self.data_lock.release()

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
                    
        while not rospy.is_shutdown():

            msg = self.socket2.recv_json()
            print(msg) 
                
            if 'head' in msg:
                par = msg['head']
                self.head.command_nod()
                print("Head nodded!")
            
            if 'kinectAngle' in msg:
                print("megvan")
                angle = msg['kinectAngle']
                print(angle)
                self.socket2.send("received")
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
