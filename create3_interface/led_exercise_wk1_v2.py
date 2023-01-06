# !/bin/python3
import rclpy
import time
import math
from create3_ros import IRobotCreate

# Initialize ros2 client library
rclpy.init()
# Establish IRobotCreate object
my_create = IRobotCreate('')

R = [0,127,255,0,127,255]
G = [255,127,0,0,127,0]
B = [0,10,0,255,127,127]

# while loop continues to run as long as ros2 client library is in ok status
while rclpy.ok:

    print("start")
    # Set RGB values for each led
    for i in range(6):
        my_create.set_ledindivcmd(R[i:] + R[:i],G[i:] + G[:i],B[i:] + B[:i]) # red
        time.sleep(1.0)

    print("finish")
    time.sleep(5.0)
