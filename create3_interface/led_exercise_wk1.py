# !/bin/python3
import rclpy
import time
import math
from create3_ros import IRobotCreate

# Initialize ros2 client library
rclpy.init()
# Establish IRobotCreate object
my_create = IRobotCreate('')


# while loop continues to run as long as ros2 client library is in ok status
while rclpy.ok:

    print("start")
    # Set RGB values for each led
    my_create.set_ledcmd(255,0,0) # red
    time.sleep(3.0)
    my_create.set_ledcmd(127,127,0)
    time.sleep(5.0)
    my_create.set_ledcmd(0,255,0) # green
    time.sleep(1.0)
    my_create.set_ledcmd(0,127,+127)
    print("finish")
    time.sleep(1.0)
    
    # spin the my_create object through one cycle
    rclpy.spin(my_create)
