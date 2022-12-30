# !/bin/python3
import rclpy
import time
import math
import threading
from create3_ros import IRobotCreate

# Initialize ros2 client library
rclpy.init()
# Establish IRobotCreate object
my_create = IRobotCreate('')

def run_loop():
    """
    run_loop function runs a simple speed and yaw rate sequence as long
    as rclpy is at status ok.
    """
    
    # Get current clock time off of Create and set it as initial time
    init_time = my_create.get_clock().now()
    
    # while loop continues to run as long as ros2 client library is in ok status
    while rclpy.ok:
        
        # duration since initial clock time (i.e. elapsed time since start)
        dur = my_create.get_clock().now() - init_time
        # convert nanosecond elapsed time into second
        t = (dur.nanoseconds)*(1e-9)

        # Set RGB values for each led
        my_create.set_ledcmd(255,0,0) # red
        time.sleep(1)
        my_create.set_ledcmd(127,127,0)
        time.sleep(1)
        my_create.set_ledcmd(0,255,0) # green
        time.sleep(1)
        my_create.set_ledcmd(0,127,127)
        time.sleep(1)
        my_create.set_ledcmd(0,0,255) # blue
        time.sleep(1)
        my_create.set_ledcmd(127,0,127)
        time.sleep(1)
        my_create.set_ledcmd(255,0,0)
        time.sleep(3)
     
        # example code to query vehicle data and print it to command line 
        #print(my_create.odom_pos)   # (1x3) The position of the robot in the odom frame
        #print(my_create.odom_eul)   # (1x3) The orientation of the robot in the odom frame as Euler Angles
        #print(my_create.odom_vel)   # (1x3) The linear velocity of the robot in the body frame
        #print(my_create.odom_omega) # (1x3) The angular velocity of the robot in the body frame

        #print(my_create.mocap_pos)  # (1x3) The Position of the robot in the mocap frame
        #print(my_create.mocap_eul)  # (1x3) The orientation of the robot in the mocap frame as Euler Angles

        #print(my_create.imu_accel)  # (1x3) gravity compensated accelerometer data from robot
        #print(my_create.imu_gyro)   # (1x3) gyro datea from robot
        #print(my_create.mouse_odom) # (1x2) Mouse sensor odometry x and y 
        #print(my_create.ir_intensity) #(1x7) Array of IR sensors from left to right
        time.sleep(0.1)


def main(args=None):    
    """ 
    Main function to run the led color changing script.
    """   
    
    # Establish a thread to run the run_loop function in parallel with other threads as needed
    thrd = threading.Thread(target=run_loop)
    
    # start the run_loop thread
    thrd.start()
    
    # spin the my_create object through one cycle
    rclpy.spin(my_create)
    # block the my_create processes until the run_loop thread has terminated
    thrd.join()
    # Delete/destroy the my_create object to disable interaction with vehicle
    my_create.destroy_node()
    # shut down the ros2 client library running under the hood
    rclpy.shutdown()


if __name__ == '__main__':
    main()