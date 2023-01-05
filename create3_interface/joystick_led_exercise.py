# !/bin/python3
import rclpy
import time
import math
from create3_ros import IRobotCreate
import pygame

# Initialize ros2 client library
rclpy.init()
# Establish IRobotCreate object
my_create = IRobotCreate('')

pygame.init()
joy = pygame.joystick.Joystick(0)
joy.init()

# while loop continues to run as long as ros2 client library is in ok status
while rclpy.ok:

    # internally process pygame event handlers
    pygame.event.pump()

    #print ("x axis: ", pygame.joystick.Joystick(0).get_axis(0))
    #print ("y axis: ", pygame.joystick.Joystick(0).get_axis(1))
    #print ("z axis: ", pygame.joystick.Joystick(0).get_axis(4))

    # make this better to scan all buttons
    but = 10;
    for x in range(joy.get_numbuttons()):
        if joy.get_button(x)==1:
            but = x

    # Write if statements to change LED color based on button press
    if but == 0:
        # Set RGB values for each led
        my_create.set_ledcmd(255,0,0) # red
        print(but)
    elif but==1:
        print(but)
        my_create.set_ledcmd(127,127,0)
    elif but==2:
        print(but)
        my_create.set_ledcmd(0,255,0) # green
    elif but==3:
        print(but)
        my_create.set_ledcmd(0,127,127)
    but = 10

    # spin the my_create object through one cycle
    rclpy.spin(my_create)