# !/bin/python3
import rclpy
import time
import math
import threading
from create3_ros import IRobotCreate

rclpy.init()
my_create = IRobotCreate('')

def run_loop():
    
    init_time = my_create.get_clock().now()
    
    while rclpy.ok:

        dur = my_create.get_clock().now() - init_time
        t = (dur.nanoseconds)*(1e-9)

        # Play C scale one note at a time
        my_create.set_audio(262,1)
        time.sleep(2)
        my_create.set_audio(294,1)
        time.sleep(2)
        my_create.set_audio(330,1)
        time.sleep(2)
        my_create.set_audio(350,1)
        time.sleep(2)
        my_create.set_audio(392,1)
        time.sleep(2)
        my_create.set_audio(440,1)
        time.sleep(2)
        my_create.set_audio(494,1)
        time.sleep(2)
        my_create.set_audio(523,1)
        time.sleep(4)

     


        #my_create.set_velcmd(spd,yaw_rate)
        #print([my_create.battery_voltage,my_create.battery_current])
        #if(t > 1.0):
        #    init_time = my_create.get_clock().now()
        #    yaw_rate = -yaw_rate
        #    spd = -spd
        
        #my_create.set_ledcmd(255,255,0)
        #print(my_create.odom_pos)   # (1x3) The position of the robot in the odom frame
        #print(my_create.odom_eul)   # (1x3) The orientation of the robot in the odom frame as Euler Angles
        #print(my_create.odom_vel)   # (1x3) The linear velocity of the robot in the body frame
        #print(my_create.odom_omega) # (1x3) The angular velocity of the robot in the body frame

        #print(my_create.mocap_pos)  # (1x3) The Position of the robot in the mocap frame
        #print(my_create.mocap_eul)  # (1x3) The orientation of the robot in the mocap frame as Euler Angles

        #print(my_create.imu_accel)  # (1x3) gravity compensated accelerometer data from robot
        #print(my_create.imu_gyro)   # (1x3) gyro datea from robot
       # print(my_create.mouse_odom) # (1x2) Mouse sensor odometry x and y 
        #print(my_create.ir_intensity) #(1x7) Array of IR sensors from left to right
        time.sleep(0.1)


def main(args=None):    
    
    
    thrd = threading.Thread(target=run_loop)

    thrd.start()

    rclpy.spin(my_create)
    thrd.join()
    my_create.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()