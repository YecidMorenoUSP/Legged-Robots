#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 17 13:27:29 2022

@author: viviansuzano
"""

# inherit from base controller
from base_controllers.base_controller import BaseController
import rospy as ros 
import numpy as np
from numpy import nan
import copy
import os

# xpp plannning message
import rosbag
from xpp_msgs.msg import RobotStateCartesian,  RobotStateCartesianTrajectory

# inverse kinematics
from base_controllers.utils.solo_ik import solo12_inverse_kinematics

# pinocchio
import pinocchio as pin

# controller
from base_controllers.utils.math_tools import *
from base_controllers.utils.common_functions import plotCoM, plotGRFs, plotJoint, plotEndeff
import matplotlib.pyplot as plt

import ex_5_2_conf_solo as conf
robotName = "solo"

class Params:
    pass 

class AdvancedController(BaseController): 

    def __init__(self):  
        BaseController.__init__(self, robot_name=robotName)
       
        #send data to param server
        self.verbose = conf.verbose                                                                                                          
        self.u.putIntoGlobalParamServer("verbose", self.verbose)
        
    def initVars(self):
        BaseController.initVars(self)	 
        
        # desired state from rosbag
        self.des_quaternion = np.zeros(4)
        self.des_basePoseW = np.zeros(6) 
        self.des_baseTwistW = np.zeros(6)
        self.des_baseAccW = np.zeros(6) 
        self.des_feetPosW = [np.zeros(3) for n in range(4)]
        self.des_feetPosB = [np.zeros(3) for n in range(4)]        
        self.des_feetVelW = [np.zeros(3) for n in range(4)]
        self.des_feetAccW = [np.zeros(3) for n in range(4)]
        self.des_feetContact = np.zeros(4)
        
        # log data
        self.des_basePoseW_log = np.empty((6, conf.buffer_size))*nan
        self.des_baseTwistW_log = np.empty((6, conf.buffer_size))*nan
        self.des_baseAccW_log = np.empty((6, conf.buffer_size))*nan
        self.des_feetPosW_log = np.empty((4, 3, conf.buffer_size))*nan
        self.des_feetVelW_log = np.empty((4, 3, conf.buffer_size))*nan
        self.des_feetContact_log = np.empty((4, conf.buffer_size))*nan
        
        self.feetPosW_log = np.empty((4, 3, conf.buffer_size))*nan
        self.feetVelW_log = np.empty((4, 3, conf.buffer_size))*nan


    def logData(self):
        if (self.log_counter < conf.buffer_size):
            BaseController.logData(self)

            self.des_basePoseW_log[:, self.log_counter] = self.des_basePoseW
            self.des_baseTwistW_log[:, self.log_counter] = self.des_baseTwistW
            self.des_baseAccW_log[:, self.log_counter] = self.des_baseAccW
            
            for leg in range(4): 
                self.des_feetPosW_log[leg, :, self.log_counter] = self.des_feetPosW[leg]
                self.des_feetVelW_log[leg, :, self.log_counter] = self.des_feetVelW[leg]
                self.feetPosW_log[leg, :, self.log_counter] = p.feetPosW[leg]
                self.feetVelW_log[leg, :, self.log_counter] = p.feetVelW[leg]
            
            self.des_feetContact_log[:, self.log_counter] = self.des_feetContact
    
    def getTrajectoryFromRosbag (self):
        bag_name = os.environ['LOCOSIM_DIR'] + "/robot_control/data/" + conf.trajectory_name + ".bag"
        traj = RobotStateCartesianTrajectory()
        traj_time = []
        
        for topic, msg, t in rosbag.Bag(bag_name).read_messages(topics=["/xpp/state_des"]):
            traj_time.append(float(msg.time_from_start.to_sec()))            
            traj.points.append(msg)
        
        print("Number of points in the trajectory = " + str(len(traj.points)))
        
        return traj, traj_time
        
    def getDesiredState(self, msg):
        
        self.des_quaternion[0] = msg.base.pose.orientation.x
        self.des_quaternion[1] = msg.base.pose.orientation.y
        self.des_quaternion[2] = msg.base.pose.orientation.z
        self.des_quaternion[3] = msg.base.pose.orientation.w
        des_euler = euler_from_quaternion(self.des_quaternion)

        self.des_basePoseW[0] = msg.base.pose.position.x
        self.des_basePoseW[1] = msg.base.pose.position.y
        self.des_basePoseW[2] = msg.base.pose.position.z
        self.des_basePoseW[3] = des_euler[0]
        self.des_basePoseW[4] = des_euler[1]
        self.des_basePoseW[5] = des_euler[2]

        self.des_baseTwistW[0] = msg.base.twist.linear.x
        self.des_baseTwistW[1] = msg.base.twist.linear.y
        self.des_baseTwistW[2] = msg.base.twist.linear.z
        self.des_baseTwistW[3] = msg.base.twist.angular.x
        self.des_baseTwistW[4] = msg.base.twist.angular.y
        self.des_baseTwistW[5] = msg.base.twist.angular.z
        
        self.des_baseAccW[0] = msg.base.accel.linear.x
        self.des_baseAccW[1] = msg.base.accel.linear.y
        self.des_baseAccW[2] = msg.base.accel.linear.z
        self.des_baseAccW[3] = msg.base.accel.angular.x
        self.des_baseAccW[4] = msg.base.accel.angular.y
        self.des_baseAccW[5] = msg.base.accel.angular.z
        
        for leg in range(4): 
            self.des_feetPosW[leg][0] = msg.ee_motion[leg].pos.x
            self.des_feetPosW[leg][1] = msg.ee_motion[leg].pos.y
            self.des_feetPosW[leg][2] = msg.ee_motion[leg].pos.z + 0.016;
            
            self.des_feetVelW[leg][0] = msg.ee_motion[leg].vel.x
            self.des_feetVelW[leg][1] = msg.ee_motion[leg].vel.y
            self.des_feetVelW[leg][2] = msg.ee_motion[leg].vel.z
            
            self.des_feetAccW[leg][0] = msg.ee_motion[leg].acc.x
            self.des_feetAccW[leg][1] = msg.ee_motion[leg].acc.y
            self.des_feetAccW[leg][2] = msg.ee_motion[leg].acc.z
            
            self.des_feetContact[leg] = msg.ee_contact[leg]   
    
    def compute_desired_qd(self, leg):
        
        # position base to feet
        w_r_bf = self.des_feetPosW[leg] - self.des_basePoseW[0:3]
        
        # base angular velocity
        des_baseAngVel_W = self.des_baseTwistW[3:6]
        
        # coriolis velocity in vase frame
        vel_coriolis_W = np.cross(des_baseAngVel_W, w_r_bf)
        
        # velocity
        vel = self.des_feetVelW[leg] - self.des_baseTwistW[0:3] - vel_coriolis_W
        qd = np.linalg.inv(self.wJ[leg]).dot(vel)     
        
        return qd
    
    def getContactJacobian (self):
        # number of feet in contact
        nc = 0;
        for leg in range(4):
            if self.des_feetContact[leg]:
                nc = nc + 1
        
        # contact jacobian
        Jc = np.zeros((3*nc,self.robot.nv))
        idx = 0
        for ee in range(4):
            leg = self.u.mapIndexToRos(ee)
            if self.des_feetContact[leg]:
                Jc[idx:idx+3,:] = self.wJf_trans[leg]
                idx = idx + 3
        
        return nc, Jc

    def computeGravityTerms (self):
        q = np.hstack((self.basePoseW[0:3], self.quaternion, self.u.mapToRos(self.q)))   
        qd = np.hstack((self.baseTwistW, self.u.mapToRos(p.qd)))
        M = self.robot.mass(q, False)
        h = self.robot.nle(q, qd, False)               
        g = self.robot.gravity(q)
        
        return g[6:]
        
    def computeTauFF (self):
        # self.robot.na = 12
        # self.robot.nv = 18
        num_tasks = 6   # base pos, base rot, 4 feet
        J = np.zeros((3*num_tasks,self.robot.nv))
        desiredAccel = np.zeros(num_tasks*3)

        idx = 0        
        
        # base position
        J[idx:idx+3,:] = self.wJb_trans
        desiredAccel[idx:idx+3] = conf.Kp_base_trans * (self.des_basePoseW[0:3] - self.basePoseW[0:3]) + \
                                  conf.Kd_base_trans * (self.des_baseTwistW[0:3] - self.baseTwistW[0:3]) + \
                                  self.des_baseAccW[0:3] - self.dJbdq_trans
        idx = idx + 3
        
        # base orientation
        
        # load math functions 
        mathJet = Math()
        # desired orientation
        Rdes = mathJet.rpyToRot(self.des_quaternion)     
        # compute orientation error
        Re = Rdes.dot(self.b_R_w.transpose())
        # express orientation error in angle-axis form                 
        orientation_err = rotMatToRotVec(Re)
        
        #conf.Kd_base_rot * (self.des_baseTwistW[3:6] - self.baseTwistW[3:6]) + \
        J[idx:idx+3,:] = self.wJb_rot
        desiredAccel[idx:idx+3] = conf.Kp_base_rot * orientation_err + \
                                  conf.Kd_base_rot * (-self.baseTwistW[3:6]) + \
                                  self.des_baseAccW[3:6] - self.dJbdq_rot
        idx = idx + 3
                                  
        # feet
        for ee in range(4):
            leg = self.u.mapIndexToRos(ee)
            J[idx:idx+3,:] = self.wJf_trans[leg]
            leg_joints = range(6+self.u.mapIndexToRos(ee)*3, 6+self.u.mapIndexToRos(ee)*3+3) 
            if self.des_feetContact[leg]:
                desiredAccel[leg_joints] = -self.dJfdq_trans[leg]
            else:
                desiredAccel[leg_joints] = conf.Kp_swingfeet * (self.des_feetPosW[leg] - self.feetPosW[leg]) + \
                                           conf.Kd_swingfeet * (self.des_feetVelW[leg] - self.feetVelW[leg]) + \
                                           self.des_feetAccW[leg] - self.dJfdq_trans[leg]
            idx = idx + 3
        
        # support consistent inverse dynamics
        print(J)
        print(desiredAccel)
        qddot = np.linalg.inv(J).dot(desiredAccel)    
        q = np.hstack((self.basePoseW[0:3], self.quaternion, self.u.mapToRos(self.q)))      
        qd = np.hstack((self.baseTwistW, self.u.mapToRos(self.qd)))
        
        M = self.robot.mass(q, False)
        M_inv = np.linalg.inv(M)
        h = self.robot.nle(q, qd, False)
        
        nc, Jc = self.getContactJacobian()
        lambda_ = np.linalg.inv(Jc.dot(M_inv).dot(Jc.T))
        
        JcTpinv = lambda_.dot(Jc).dot(M_inv)
        Nc = np.eye(self.robot.nv) - Jc.T.dot(JcTpinv)
        
        S = np.zeros((self.robot.na, self.robot.nv))
        S[:,6:] = np.eye(self.robot.na)        
        
        NcSTinv = np.linalg.pinv(Nc.dot(S.T))
        tau_ff = NcSTinv.dot(Nc).dot(M.dot(qddot) + h)
        
        return tau_ff
     

def talker(p):
    
    traj, traj_time = p.getTrajectoryFromRosbag()    
    traj_idx = 0
    
    p.start()
    p.initVars()          
    p.startupProcedure() 
    p.pid.setPDs(conf.kp, conf.kd, conf.ki) 
    
    rate = ros.Rate(1/conf.dt)
    print("Loop frequency: " + str(1/conf.dt) + "Hz")
    
    base_goal = [traj.points[-1].base.pose.position.x, 
                 traj.points[-1].base.pose.position.y, 
                 traj.points[-1].base.pose.position.z]

    # Control loop               
    while (p.time  < conf.exp_duration) or conf.CONTINUOUS:
        #print("Time: " + str(p.time) + ", trajectory index: " + str(traj_idx))    
        
        #update the kinematics
        p.updateKinematics()
                                
        # EXERCISE 1: Compute desired joint position and velocities for the trajectory
        p.getDesiredState(traj.points[traj_idx])
        for leg in range(4):
            idx = 3*leg
            p.qd_des[idx:idx+3] = p.compute_desired_qd(leg)
            p.des_feetPosB[leg] = p.des_feetPosW[leg] - p.des_basePoseW[0:3]
    
        des_feet_pos = np.vstack((p.des_feetPosB[0], p.des_feetPosB[1], p.des_feetPosB[2], p.des_feetPosB[3]))
        p.q_des = solo12_inverse_kinematics(des_feet_pos)
        
        # EXERCISE 2: support consistent inverse dynamics
        #p.tau_ffwd = np.zeros(p.robot.na)
        #p.tau_ffwd = p.u.mapFromRos(p.computeGravityTerms());
        #p.tau_ffwd = p.u.mapFromRos(p.h_joints);
        p.tau_ffwd = p.u.mapFromRos(p.computeTauFF())
        
        # send desired command to the ros controller  
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
                        
        if traj_idx < (len(traj.points)-1) :
            traj_idx = traj_idx + 1
            p.logData() 
        
        # plot actual contact forces 
        for leg in range(4):
            p.ros_pub.add_arrow(p.feetPosW[leg], p.u.getLegJointState(leg, p.grForcesW/(5*p.robot.robotMass())),"green") 
#            p.ros_pub.add_marker(p.des_feetPosW[leg], 0.02, "green") 
#            p.ros_pub.add_marker(p.feetPosW[leg], 0.02, "blue")
        p.ros_pub.add_marker(base_goal, 0.05, "red") 
        p.ros_pub.publishVisual() 
        
        p.time = p.time + conf.dt
        
        # wait for synconization of the control loop
        rate.sleep()       
        # stops the while loop if  you prematurely hit CTRL+C                    
        if ros.is_shutdown():
            print ("Shutting Down")                    
            break;                                                
                             
    # restore PD when finished        
    #p.pid.setPDs(6.0, 0.2, 0.0) 
    ros.sleep(1.0)                
    print ("Shutting Down")                 
    ros.signal_shutdown("killed")           
    p.deregister_node()        
    
    for leg in range(4):           
        plotEndeff('position', leg, p.time_log, p.feetPosW_log[leg], p.des_feetPosW_log[leg], None, None, None, None, None, conf.ee_names[leg])
        #plotEndeff('velocity', leg+4, p.time_log, None, None, p.feetVelW_log[leg], p.des_feetVelW_log[leg], None, None, None, conf.ee_names[leg])
    
#    plotCoM('position', 8, p.time_log, p.des_basePoseW_log, p.basePoseW_log, p.des_baseTwistW_log, p.baseTwistW_log, None, None)
    plotJoint('position', 9, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log)
#    plotJoint('velocity', 10, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log)
    plotJoint('torque', 11, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log)
    plt.show(block=True)
    
if __name__ == '__main__':
    p = AdvancedController()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    
        
