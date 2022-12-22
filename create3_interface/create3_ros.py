import numpy as np
import math
import quaternion

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from irobot_create_msgs.msg import *

red_code = LedColor(red=255,green=0,blue=0)
orange_code = LedColor(red=255,green=0,blue=0)
white_code = LedColor(red=255,green=255,blue=255)
blue_code =  LedColor(red=0,green=0,blue=255)
cyan_code =  LedColor(red=0,green=255,blue=255)

qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

class IRobotCreate(Node):

    def __init__(self,robot_name):
        super().__init__('irobot_create_interface')
        
        # Define Status Flags
        self.is_armed = False
        self.is_stopped = True
        self.is_slipping = False
        self.is_carried = False
        self.is_docked = False
        self.dock_visible = False
        self.wheels_enabled = False

        # Variables for IMU 
        self.imu_accel = np.zeros(3)
        self.imu_gyro = np.zeros(3)
        self.imu_quat = np.quaternion(1,0,0,0)
        self.imu_eul = np.zeros(3)

        self.mocap_pos = np.zeros(3)
        self.mocap_quat = np.quaternion(1,0,0,0)
        self.mocap_eul = np.zeros(3)

        self.odom_pos = np.zeros(3)
        self.odom_quat = np.quaternion(1,0,0,0)
        self.odom_eul = np.zeros(3)
        self.odom_vel = np.zeros(3)
        self.odom_omega = np.zeros(3)

        self.wheel_ticks = np.zeros(2)
        self.wheel_vel = np.zeros(2)

        self.motor_current = np.zeros(2)
        self.motor_pwm = np.zeros(2)

        self.ir_intensity = np.zeros(7)
        self.mouse_odom = np.zeros(2)
        self.battery_voltage = 0
        self.battery_temp = 0
        self.battery_current = 0
        self.battery_percentage = 0

        self.vel_cmd = Twist()

        self.name_prefix = robot_name
    
        # Set up Publishers for create commands
        self.cmd_vel_pub = self.create_publisher(Twist, self.name_prefix+'/cmd_vel', qos_profile)
        self.led_pub = self.create_publisher(LightringLeds,self.name_prefix+'/cmd_lightring', qos_profile)
        self.audio_pub = self.create_publisher(AudioNoteVector,self.name_prefix+'/cmd_audio', qos_profile) 

        # Set up Sensor Subscribers
        self.ir_sens_sub = self.create_subscription(IrIntensityVector,self.name_prefix+'/ir_intensity',self.ir_intensity_callback,qos_profile)
        self.imu_sub = self.create_subscription(Imu,self.name_prefix+'/imu',self.imu_callback,qos_profile)
        self.odom_sub = self.create_subscription(Odometry,self.name_prefix+'/odom',self.odom_callback,qos_profile)
        self.wheel_tick_sub = self.create_subscription(WheelTicks,self.name_prefix+'/wheel_ticks',self.wheeltick_callback,qos_profile)
        self.wheel_vel_sub = self.create_subscription(WheelVels,self.name_prefix+'/wheel_vels',self.wheelvels_callback,qos_profile)
        self.mocap_sub = self.create_subscription(PoseStamped,self.name_prefix+'/pose',self.mocap_callback,qos_profile)


        # Se up Status Subscribers
        self.batt_sub = self.create_subscription(BatteryState,self.name_prefix+'/battery_state',self.battery_callback,qos_profile)
        self.button_sub = self.create_subscription(InterfaceButtons,self.name_prefix+'/interface_buttons',self.button_callback,qos_profile)
        self.mouse_sub = self.create_subscription(Mouse,self.name_prefix+'/mouse',self.mouse_callback,qos_profile)
        self.wheel_status_sub = self.create_subscription(WheelStatus,self.name_prefix+'/wheel_status',self.wheel_status_callback,qos_profile)
        self.hazard_Sub = self.create_subscription(HazardDetectionVector,self.name_prefix+'/hazard_detection',self.hazard_callback,qos_profile)
        self.kidnap_sub = self.create_subscription(KidnapStatus,self.name_prefix+'/kidnap_status',self.kidnap_callback,qos_profile)
        self.slip_sub = self.create_subscription(SlipStatus,self.name_prefix+'/slip_status',self.slip_callback,qos_profile)
        self.stop_sub = self.create_subscription(StopStatus,self.name_prefix+'/stop_status',self.stop_callback,qos_profile)
        self.dock_sub = self.create_subscription(Dock,self.name_prefix+'/dock',self.dock_callback,qos_profile)

        timer_period = 0.1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def imu_callback(self,msg):
        self.imu_accel[0]= msg.linear_acceleration.x
        self.imu_accel[1] = msg.linear_acceleration.y
        self.imu_accel[2] = msg.linear_acceleration.z

        self.imu_gyro[0]= msg.angular_velocity.x
        self.imu_gyro[1] = msg.angular_velocity.y
        self.imu_gyro[2] = msg.angular_velocity.z

        self.imu_quat = np.quaternion(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z)

        R = quaternion.as_rotation_matrix(self.imu_quat)

        self.imu_eul[2] =  math.atan2(R[1][0],R[0][0])
        self.imu_eul[1] = -math.asin(R[2][0])
        self.imu_eul[0] = math.atan2(R[2][1],R[2][2])


    def mocap_callback(self,msg):
        self.mocap_pos[0] = msg.pose.position.x
        self.mocap_pos[1] = msg.pose.position.y
        self.mocap_pos[2] = msg.pose.position.z

        self.mocap_quat = np.quaternion(msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z) 
        R = quaternion.as_rotation_matrix(self.mocap_quat)

        self.mocap_eul[2] =  math.atan2(R[1][0],R[0][0])
        self.mocap_eul[1] = -math.asin(R[2][0])
        self.mocap_eul[0] = math.atan2(R[2][1],R[2][2])        

    def odom_callback(self,msg):
        self.odom_pos[0] = msg.pose.pose.position.x
        self.odom_pos[1] = msg.pose.pose.position.y
        self.odom_pos[2] = msg.pose.pose.position.z

        self.odom_quat = np.quaternion(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z) 
        R = quaternion.as_rotation_matrix(self.odom_quat)

        self.odom_eul[2] =  math.atan2(R[1][0],R[0][0])
        self.odom_eul[1] = -math.asin(R[2][0])
        self.odom_eul[0] = math.atan2(R[2][1],R[2][2])  

        self.odom_vel[0] = msg.twist.twist.linear.x
        self.odom_vel[1] = msg.twist.twist.linear.y
        self.odom_vel[2] = msg.twist.twist.linear.z

        self.odom_omega[0] = msg.twist.twist.angular.x
        self.odom_omega[1] = msg.twist.twist.angular.y
        self.odom_omega[2] = msg.twist.twist.angular.z  
    
    def wheeltick_callback(self,msg):
        self.wheel_ticks[0] = msg.ticks_left
        self.wheel_ticks[1] = msg.ticks_right

    def wheelvels_callback(self,msg):
        self.wheel_vel[0] = msg.velocity_left
        self.wheel_vel[1] = msg.velocity_right

    def wheel_status_callback(self,msg):
        self.wheels_enabled = msg.wheels_enabled
        self.motor_pwm[0] = msg.pwm_left
        self.motor_pwm[1] = msg.pwm_right
        self.motor_current[0] = msg.current_ma_left
        self.motor_current[1] = msg.current_ma_right

    def kidnap_callback(self,msg):
        self.is_carried = msg.is_kidnapped

    def dock_callback(self,msg):
        self.dock_visible = msg.dock_visible
        self.is_docked = msg.is_docked
    
    def slip_callback(self,msg):
        self.is_slipping = msg.is_slipping

    def stop_callback(self,msg):
        self.is_stopped = msg.is_stopped

    def hazard_callback(self,msg):
        msg
        #for detect in msg.detections:
            #print detect

    def ir_intensity_callback(self,msg):
        #print(msg)
        for i in range(7):
            self.ir_intensity[i] = msg.readings[i].value

    def mouse_callback(self,msg):
        self.mouse_odom[0] = msg.integrated_x
        self.mouse_odom[1] = msg.integrated_y

    def battery_callback(self,msg):
        self.battery_voltage = msg.voltage
        self.battery_temp = msg.temperature
        self.battery_current = msg.current
        self.battery_percentage = msg.percentage


    def button_callback(self,msg):

        if(msg.button_2.is_pressed):
            self.is_armed = not self.is_armed

    def set_audio(self,freq,dur):

        secs = math.floor(dur)
        nsecs = math.floor((dur-secs)*1e9)

        audio_msg = AudioNoteVector()
        audio_msg.append = True
        note = AudioNote()
        note.frequency = freq
        note.max_runtime.sec = secs
        note.max_runtime.nanosec = nsecs        
        
        audio_msg.notes.append(note)

        self.audio_pub.publish(audio_msg)

    def set_ledcmd(self,R,G,B):
        color = LedColor(red=R,green=G,blue=B)
        ring = LightringLeds()
        for i in range(6):
            ring.leds[i]= color

        ring.override_system = True
        self.led_pub.publish(ring)

    def set_velcmd(self,vel,yaw_rate):
        cmd_msg = Twist()
        cmd_msg.linear.x = vel
        cmd_msg.angular.z = yaw_rate    
        self.cmd_vel_pub.publish(cmd_msg)
    
    def set_armed(self,val):
        self.is_armed = val

    def timer_callback(self):
        
        if(self.is_armed):
            
            self.cmd_vel_pub.publish(self.vel_cmd)