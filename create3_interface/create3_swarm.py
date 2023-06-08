#!/bin/python3
from functools import partial
import numpy as np
import quaternion
import math
import time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from create3_ros import IRobotCreate

red = [255,0,0]
orange = [255,127,0]
green = [0,255,0]
blue = [0,0,255]
white = [255,255,255]
magenta = [255,0,255]
cyan = [0,255,255]
yellow = [255,255,0]

class createSwarm(IRobotCreate):

    def __init__(self,robot_name,team_names):
        IRobotCreate.__init__(self,robot_name)

        self.mode = 0
        self.last_mode = 0
        self.armed = False 
        self.names = team_names
        self.path_received = False
        self.wp_received = False
        self.timeout = self.get_clock().now()
        self.color_map = [red,blue,green,orange]

        self.team_colors = [white,cyan,yellow,magenta]

        self.is_leader = False
        self.robot_index = 0
        self.robot_name = robot_name
        self.toggle = True

        # If robot name is first in the list it is the leader
        if(robot_name == self.names[0]): # 
            self.is_leader = True

        self.robot_color = self.names.index(robot_name)

        self.speed_scale = 0.7
        self.yaw_rate_scale = 1.5
        self.wp_radius = 0.15

        self.names_n = len(self.names)
        self.sub_list = []
    

        self.odom_list = []
        self.team_pos = []
        self.team_eul = []
        self.team_vel = [] # Body frame velocity from Odom
        self.team_vel_g  = [] #Global frame veloicty from Odom
        self.team_omega = []

        self.team_wp = []
        self.team_path = []
        self.team_des_pose = []
        self.team_des_vel = []

        self.teleop_spd = 0.0
        self.teleop_yawrate = 0.0

        self.get_clock().now()

        self.joy_sub = self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.wp_sub = self.create_subscription(PoseStamped,'/goal_pose',self.wp_callback,10)
        self.traj_sub = self.create_subscription(Float32MultiArray,'/goal_trajectory',self.trajectory_callback,10)

        for current_name in self.names:
            sub = self.create_subscription(Odometry,'/'+current_name+'/mocap/odom',partial(self.team_odom_callback,current_name),10)
            self.sub_list.append(sub)
            pos = np.zeros([1,3])
            self.team_pos.append(pos)
            eul = np.zeros([1,3])
            self.team_eul.append(eul)
            vel = np.zeros([1,3])
            self.team_vel.append(vel)
            velg = np.zeros([1,3])
            self.team_vel_g.append(velg)
            omega = np.zeros([1,3])
            self.team_omega.append(omega)


        self.timer = self.create_timer(0.5, self.update_callback)
                

    def wp_callback(self,msg):

        quat = np.quaternion(msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z)
        R = quaternion.as_rotation_matrix(quat)
        wp_eul = np.array([math.atan2(R[2][1],R[2][2]),-math.asin(R[2][0]),math.atan2(R[1][0],R[0][0])])    

        self.team_wp = np.array([msg.pose.position.x,msg.pose.position.y,wp_eul[2]])
        self.wp_received = True

    def trajectory_callback(self,msg):

        des_yaw = math.atan2(msg.data[4],msg.data[3])
        self.team_des_pose = np.array([msg.data[0],msg.data[1],msg.data[2]]) #Inertial Frame (x,y,psi)
        self.team_des_vel = np.array([msg.data[3],msg.data[4],msg.data[5]]) # Inertial Frame (vx,vy,r)
#        self.team_traj = np.array([msg.data[0],msg.data[1],msg.data[3],msg.data[4]])

        # Get current clock time off of Create and set it as initial time
        self.timeout = self.get_clock().now()

        self.path_received = True        

    def team_odom_callback(self,name,msg):
        k = self.names.index(name)
        self.team_pos[k][0,:] = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])

        quat = np.quaternion(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        R = quaternion.as_rotation_matrix(quat)
        self.team_eul[k][0,:] = np.array([math.atan2(R[2][1],R[2][2]),-math.asin(R[2][0]),math.atan2(R[1][0],R[0][0])])

        self.team_vel[k][0,:] = np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z])
        
        vel_g = quat*np.quaternion(0,msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z)*np.conjugate(quat)

        self.team_vel_g[k][0,:] = np.array([vel_g.x,vel_g.y,vel_g.z])

        self.team_omega[k][0,:] = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

 

    def joy_callback(self,msg):

        if(msg.buttons[0]): # Idle
            self.mode = 0

        if(msg.buttons[1]): # Manual
            self.mode = 1

        if(msg.buttons[3]): # Auto
            self.mode = 2    
        
        if(msg.buttons[4]): # Reset
            self.mode = 3

        if(msg.buttons[7]):

            if(self.toggle): 
                self.robot_index+=1
                self.robot_index %= self.names_n
                self.toggle = False
            
        if(msg.buttons[6]):

            if(self.toggle): 
                self.robot_index-=1
                self.robot_index %= self.names_n
                self.toggle = False

            
            self.toggle_timeout = self.get_clock().now()



        self.teleop_spd =  msg.axes[1]*self.speed_scale
        self.teleop_yawrate = msg.axes[2]*self.yaw_rate_scale                   
    
    def update_callback(self):
        
        self.toggle = True  
        dur = self.get_clock().now() - self.timeout
        t = (dur.nanoseconds)*(1e-9)
        if(t>1.0):
            self.path_received = False

        if(self.mode != self.last_mode):
            super().set_audio(100+self.mode*100,0.1)
            if(self.mode == 1):
                self.robot_index = 0
                      

        if(self.mode ==0):
            super().set_ledcmd(self.color_map[self.mode][0],self.color_map[self.mode][1],self.color_map[self.mode][2])

        if(self.mode == 1):
            if(self.robot_name == self.names[self.robot_index]): # 
                super().set_ledcmd(self.color_map[self.mode][0],self.color_map[self.mode][1],self.color_map[self.mode][2])
                time.sleep(0.1)
                super().set_ledcmd(0,0,0)
                time.sleep(0.1)
                super().set_ledcmd(self.color_map[self.mode][0],self.color_map[self.mode][1],self.color_map[self.mode][2])
                time.sleep(0.1)
                super().set_ledcmd(0,0,0)                
            else:
                super().set_ledcmd(self.color_map[self.mode][0],self.color_map[self.mode][1],self.color_map[self.mode][2])            


        if(self.mode == 2):
            super().set_ledcmd(self.team_colors[self.robot_color][0],self.team_colors[self.robot_color][1],self.team_colors[self.robot_color][2])
            time.sleep(0.1)
            super().set_ledcmd(0,0,0)
            time.sleep(0.1)
            super().set_ledcmd(self.team_colors[self.robot_color][0],self.team_colors[self.robot_color][1],self.team_colors[self.robot_color][2])
            time.sleep(0.1)
            super().set_ledcmd(0,0,0)

        if(self.mode == 3):
            super().set_ledcmd(self.color_map[self.mode][0],self.color_map[self.mode][1],self.color_map[self.mode][2])
            time.sleep(0.1)
            super().set_ledcmd(0,0,0)
            time.sleep(0.1)
            super().set_ledcmd(self.color_map[self.mode][0],self.color_map[self.mode][1],self.color_map[self.mode][2])
            time.sleep(0.1)
            super().set_ledcmd(0,0,0)    

         

        self.last_mode = self.mode
