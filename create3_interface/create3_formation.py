# !/bin/python3
import sys
import rclpy
import time
import math
import numpy as np
import threading
from create3_swarm import createSwarm

def wrapToPi(ang):

    while(ang > math.pi):
        ang -= 2*math.pi

    while(ang < -math.pi):
        ang += 2*math.pi

    return ang

def saturation(val,max,min):
    if(val > max):
        val = max

    if(val < min):
        val = min

    return val

def pot_col_avoid(safeR, outR, dist_ij):
    # ca = 1e-6
    if (dist_ij > safeR and dist_ij < outR):
        potVal_ij  = 2*(pow(outR,2) - pow(safeR,2))/(pow(dist_ij,2)-pow(safeR,2))/(pow(dist_ij,2)-pow(outR,2))
        # potVal_ij  = 4*ca*(pow(outR,2) - pow(safeR,2))*(pow(dist_ij,2)-pow(outR,2))/pow(pow(dist_ij,2)-pow(safeR,2),3)
    else:
        potVal_ij = 0
        
    return potVal_ij

class swarmNode(createSwarm):
    def __init__(self,robot_name):
        # Establish IRobotCreate object
        self.robot = robot_name
#        self.team = ['create_1','create_2','create_3','create_4']
        self.team = ['create_4','create_1','create_2']

        #self.team = ['create_1']

        #self.team = ['create_1']

        createSwarm.__init__(self,robot_name,self.team)
        # Initialize ros2 client library

        #self.declare_parameter("robot_name")
        #self.declare_parameter("team_names")
        self.home_poses = np.zeros([4,3])
        self.home_poses[0,:] = np.array([0,0,0])
        self.home_poses[1,:] = np.array([1.5,0.0,0])
        self.home_poses[2,:] = np.array([1.5,1.5,0])
        self.home_poses[3,:] = np.array([0,1.5,0])

        self.my_index = self.team.index(robot_name)


        # team = ['create_1','create_2','create_3','create_4']1.0


        self.wp_rad = 0.35
        self.cruise_spd = 1
        self.Ka = 0.6
        # Gain Matricies {x,y,theta}
        self.Kp = np.diag([5.0,5.0,1.0])
        self.Kv = np.diag([0.75,0.75,1.0])      
        self.Cp = np.diag([2.5,2.5,1.5])
        self.Cv = np.diag([0.8,0.8,0.8])

        #self.Kp = np.diag([1.5,1.5,1.5])
        #self.Kv = np.diag([0.5,0.5,0.5])      
        #self.Cp = np.diag([0.5,0.5,0.5])
        #self.Cv = np.diag([0.5,0.5,0.5])        
        self.Kyaw = 1.5
        self.L = 0.15
        self.sens_rad = 0.5
        self.avoid_rad = 0.2
        self.eta = 3
        self.sigma = 0*0.75 # std deviation of yaw_rate disturbance
        self.psi_dist = 0
        self.w  = 0
#        self.psi_dist += - self.w*self.eta*self.dt + np.sqrt(self.dt)*self.sigma*np.random.randn()  

        #self.is_leader = False # Give All robots acces to the trajectory

        self.formation = {}
        self.formation[self.team[0]]= np.zeros(3)
        self.formation[self.team[1]]= np.array([1.0,0,0])
        self.formation[self.team[2]]= np.array([1.0,1.0,0])
  #      self.formation[self.team[3]]= np.array([0.0,1.0,0])        

                
        self.laplacian = {}
        self.laplacian[self.team[0]] = 0*np.array([0,1,0,0])
        self.laplacian[self.team[1]] = np.array([1,0,1,0])
        self.laplacian[self.team[2]] = np.array([1,1,0,0])
 #       self.laplacian[self.team[3]] = np.array([1,0,1,0]) 
        self.dt = 0.1       
        self.timer = self.create_timer(self.dt, self.control_update)



    def control_update(self):    

        # Get current clock time off of Create and set it as initial time
        init_time = self.get_clock().now()
                   
        # duration since initial clock time (i.e. elapsed time since start)
        dur = self.get_clock().now() - init_time
        # convert nanosecond elapsed time into second
        t = (dur.nanoseconds)*(1e-9)
        
        # invoke the set_velcmd method of the my_create object to set the speed and yaw_rate

        if(self.mode == 1):
            
            if(self.robot_name == self.names[self.robot_index]):
                self.set_velcmd(self.teleop_spd,self.teleop_yawrate)
            else:
                self.set_velcmd(0.0,0.0)

        elif(self.mode == 2):
 
            if(self.path_received):     

                u_cmd = np.zeros(3)
                vel_cmd = np.zeros(3)
                
                err = np.zeros(3)
                ctrl_pnt = np.array([self.L*math.cos(self.mocap_odom_eul[2]), self.L*math.sin(self.mocap_odom_eul[2])])
          
                ii = 0
                sum_err = np.zeros(3)
                for name in self.team:
                    lead_pose = np.zeros(3)
                    lead_vel = np.zeros(3)
                    lead_eul = np.zeros(3)
                    offset = np.zeros(3)
                    ind = self.team.index(name)
                    

                    lead_pose = self.team_pos[ind]
                    lead_vel = self.team_vel_g[ind] 
                    lead_eul = self.team_eul[ind]
                    lead_omega = self.team_omega[ind]           
                    offset = self.formation[self.robot] - self.formation[name]

                    

                    form_err = np.array([self.mocap_odom_pos[0],self.mocap_odom_pos[1]]) - (lead_pose[0][0:2]+offset[0:2])

                    #form_yaw = math.atan2(-form_err[1],-form_err[0])
                    #form_yaw_err = wrapToPi(self.mocap_odom_eul[2]-form_yaw)
                    form_yaw_err_2 = wrapToPi(self.mocap_odom_eul[2]-lead_eul[0][2])

                    dr = math.sqrt(form_err[0]**2 + form_err[1]**2)
                    dr = saturation(dr,1,0)

                    avoid_err = self.mocap_odom_pos[0:2] - lead_pose[0][0:2]                    
                    avoid_dist = np.sqrt(avoid_err[0]**2 + avoid_err[1]**2)
                    

                    if(self.is_leader):
                        mu_x = self.laplacian[self.robot][ii]*(-self.Kp[0][0]*(form_err[0]) - self.Kv[0][0]*(self.mocap_odom_vel_g[0] - lead_vel[0][0])) - self.Cv[0][0]*(self.mocap_odom_vel_g[0] - self.team_des_vel[0])- 0*pot_col_avoid(self.avoid_rad,self.sens_rad,avoid_dist)*avoid_err[0]
                        mu_y = self.laplacian[self.robot][ii]*(-self.Kp[1][1]*(form_err[1]) - self.Kv[1][1]*(self.mocap_odom_vel_g[1] - lead_vel[0][1])) - self.Cv[1][1]*(self.mocap_odom_vel_g[1] - self.team_des_vel[1]) - 0*pot_col_avoid(self.avoid_rad,self.sens_rad,avoid_dist)*avoid_err[1]                        
                        cmd_yaw = math.atan2(mu_y,mu_x)
                        form_yaw_err = wrapToPi(self.mocap_odom_eul[2]-cmd_yaw)                        
                        mu_r = self.laplacian[self.robot][ii]*(-self.Kp[2][2]*form_yaw_err  - 0*self.Kv[2][2]*(self.mocap_odom_omega[2] - lead_omega[0][2])) - self.Cv[2][2]*(self.odom_omega[2]-self.team_des_vel[2])

                    else:
                        mu_x = self.laplacian[self.robot][ii]*(-self.Kp[0][0]*(form_err[0]) - self.Kv[0][0]*(self.mocap_odom_vel_g[0] - lead_vel[0][0])) - self.Cv[0][0]*(self.mocap_odom_vel_g[0] - self.team_des_vel[0]) - 0*pot_col_avoid(self.avoid_rad,self.sens_rad,avoid_dist)*avoid_err[0]
                        mu_y = self.laplacian[self.robot][ii]*(-self.Kp[1][1]*(form_err[1]) - self.Kv[1][1]*(self.mocap_odom_vel_g[1] - lead_vel[0][1])) - self.Cv[1][1]*(self.mocap_odom_vel_g[1] - self.team_des_vel[1]) -0*pot_col_avoid(self.avoid_rad,self.sens_rad,avoid_dist)*avoid_err[1]
                        cmd_yaw = math.atan2(mu_y,mu_x)
                        form_yaw_err = wrapToPi(self.mocap_odom_eul[2]-cmd_yaw)
                        mu_r = self.laplacian[self.robot][ii]*(-self.Kp[2][2]*form_yaw_err - 0*self.Kp[2][2]*form_yaw_err_2*(1-dr) - 0*self.Kv[2][2]*(self.mocap_odom_omega[2] - lead_omega[0][2])) - self.Cv[2][2]*(self.odom_omega[2]-self.team_des_vel[2])
                        print("euler: ",self.mocap_odom_eul[2], " cmd: ", cmd_yaw)


                   # if(self.laplacian[self.robot][ii]==1):
                    #    print("robot cmd: ",mu_x,mu_y, mu_r)

                    u_cmd += np.array([mu_x,mu_y,mu_r])
                    

                    sum_err += form_err[0]
                    ii+=1

                if(self.is_leader):
                    err[0] =  self.mocap_odom_pos[0] + 0*ctrl_pnt[0] - self.team_des_pose[0] 
                    err[1] =  self.mocap_odom_pos[1] + 0*ctrl_pnt[1] - self.team_des_pose[1]
#                    curr_err[2] = wrapToPi(self.mocap_odom_eul[2]-self.team_des_pose[2])

                    wp_psi = math.atan2(-err[1],-err[0])

                    psi_err = wrapToPi(self.mocap_odom_eul[2]-wp_psi)
                    
                    self.w += - self.w*self.eta*self.dt + np.sqrt(self.dt)*self.sigma*np.random.randn()  

                    self.psi_dist = self.psi_dist + self.w*self.dt  

                    psi_err_2 = wrapToPi(self.mocap_odom_eul[2]-wrapToPi(self.team_des_pose[2]+self.psi_dist))
                    ux = -self.Cp[0][0]*err[0] #- self.Cv[0][0]*(self.mocap_odom_vel_g[0] - self.team_des_vel[0])
                    uy = -self.Cp[1][1]*err[1] #- self.Cv[1][1]*(self.mocap_odom_vel_g[1] - self.team_des_vel[1])
                    ur = -self.Cp[2][2]*psi_err - self.Cp[2][2]*psi_err_2 #- self.Cv[2][2]*(self.odom_omega[2]-self.team_des_vel[2])
                    #w[k+1] = w[k] + eta*(wStar - w[k])*dt + np.sqrt(dt)*sigma*np.random.randn()  

                   # print("error des_yaw and yaw_dist:",math.sqrt(err[0]**2 + err[1]**2), self.psi_dist)

                  #  print("error des_yaw and yaw_dist:",math.sqrt(err[0]**2 + err[1]**2), psi_err)
                    u_cmd += np.array([ux,uy,ur])

                else:
                    err = sum_err

                R_gb = np.array([[math.cos(self.mocap_odom_eul[2]),math.sin(self.mocap_odom_eul[2]),0],[-math.sin(self.mocap_odom_eul[2]),math.cos(self.mocap_odom_eul[2]),0],[0,0,1]])
                u_cmd_b = np.dot(R_gb,u_cmd)
                vel_cmd = np.array([self.odom_vel[0],self.odom_vel[1],self.odom_omega[2]]) + u_cmd_b*self.dt 
                                                    
                dist = np.sqrt(err[0]**2 + err[1]**2)                
                cmd_spd = vel_cmd[0]
                cmd_yaw_rate = vel_cmd[2]


                cmd_spd = saturation(cmd_spd,0.2,-0.2)
                cmd_yaw_rate = saturation(cmd_yaw_rate,0.8,-0.8)

                #slow_scale = abs(cmd_spd)/abs(cmd_yaw_rate)
                #3slow_scale = saturation(slow_scale,0.5,1.0)
                #cmd_spd = cmd_spd*slow_scale

                #print(cmd_spd,cmd_yaw_rate)
                #if(dist > self.wp_radius):
                #if(True):
                self.set_velcmd(cmd_spd,cmd_yaw_rate)
                #else:
                #self.set_velcmd(0.0,0.0)

                      
            else:
                print("waiting for waypoint")

        elif(self.mode == 3):

            print("reset mode")               
            vel_cmd = np.zeros(3)
            err = np.zeros(3)
            avoid_cmd = np.zeros(3)
            ctrl_pnt = np.array([self.L*math.cos(self.mocap_odom_eul[2]), self.L*math.sin(self.mocap_odom_eul[2]),0])
            err =  self.home_poses[self.my_index,:] - self.mocap_odom_pos + ctrl_pnt

            des_yaw = math.atan2(err[1],err[0])
            
            err_yaw = wrapToPi(des_yaw- self.mocap_odom_eul[2])


            cmd_yaw_rate = self.Kyaw*err_yaw
            cmd_spd = 0.0
            dist = np.sqrt(err[0]**2 + err[1]**2)
            if(dist > 0.05):

                if(abs(err_yaw)<0.1):
                    cmd_spd = 0.5*dist

            else:
                cmd_spd = 0.0
                cmd_yaw_rate = 0.0                
            
            self.set_velcmd(cmd_spd,cmd_yaw_rate)

        else:
            self.set_velcmd(0.0,0.0)
        



def main(args):    
    
    rclpy.init()    
    # spin the my_create object through one cycle
    swarm_node = swarmNode(args)
    rclpy.spin(swarm_node)

    # Delete/destroy the my_create object to disable interaction with vehicle
    swarm_node.destroy_node()
    # shut down the ros2 client library running under the hood
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1])