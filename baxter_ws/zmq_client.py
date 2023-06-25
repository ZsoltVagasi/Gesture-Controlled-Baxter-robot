import sys
import zmq
import time 

port = "15557"

# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.subscribe("")

socket.connect("tcp://192.168.98.219:15557")
# socket.connect("tcp://192.168.98.212:15557")

while True:
    received = socket.recv_json()
    print(received)
    # time.sleep(1)