#common stuff 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm
import eigenpy

import os
from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub

import ex_2_conf as conf

FLAG = {}

FLAG['SHOW_ANIMATION'] = False
FLAG['SIN_WAVE'] = False
FLAG['SQUARE_WAVE'] = False
FLAG['PD_CONTROL'] = False
FLAG['CRITICAL_DAMPING'] = False
FLAG['GRAVITY_COMPENSATION'] = False
FLAG['FEED_FOWARD'] = False
FLAG['EXTERNAL_FORCE'] = False
FLAG['POINTS'] = ['1.1','1.2','1.3','1.3.1','1.3.2','1.4','1.5','1.6','1.7']
FLAG['POINT'] = '1.5'

#for POINT in FLAG['POINTS']:
#    FLAG['POINT'] = POINT
    

    
POINT = FLAG['POINT']
if(POINT == '1.1'):
    FLAG['SIN_WAVE'] = True
elif(POINT == '1.2'):
    FLAG['SQUARE_WAVE'] = True
elif(POINT == '1.3'):
    FLAG['SQUARE_WAVE'] = True
    FLAG['PD_CONTROL'] = True
    conf.kp = np.eye(6)*300
    conf.kd = np.eye(6)*20
elif(POINT == '1.3.1'):
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL'] = True
    conf.kp = np.eye(6)*300
    conf.kd = np.eye(6)*20
elif(POINT == '1.3.2'):
    FLAG['SQUARE_WAVE'] = True
    FLAG['PD_CONTROL'] = True
    conf.kp = np.eye(6)*300
    conf.kd = np.eye(6)*20
elif(POINT == '1.4'):
    FLAG['SQUARE_WAVE'] = True
    FLAG['PD_CONTROL'] = True
    conf.kp = np.eye(6)*600
    conf.kd = np.eye(6)*30
    conf.dt = 0.0001
elif(POINT == '1.4.1'):
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL'] = True
    conf.kp = np.eye(6)*600
    conf.kd = np.eye(6)*30
    conf.dt = 0.0001
elif(POINT == '1.4.2'):
    FLAG['SQUARE_WAVE'] = True
    FLAG['PD_CONTROL'] = True
    conf.kp = np.eye(6)*600
    conf.kd = np.eye(6)*30
    conf.dt = 0.0001
elif(POINT == '1.5'):
    FLAG['SQUARE_WAVE'] = True
    FLAG['PD_CONTROL'] = True
    FLAG['CRITICAL_DAMPING'] = True
    conf.kp = np.eye(6)*300
    conf.kd = np.eye(6)*20
elif(POINT == '1.6'):
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL'] = True
    FLAG['GRAVITY_COMPENSATION'] = True
elif(POINT == '1.7'):
    conf.kp = np.eye(6)*300
    conf.kd = np.eye(6)*40
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL'] = True
    FLAG['GRAVITY_COMPENSATION'] = True
    FLAG['FEED_FOWARD'] = True
    FLAG['EXTERNAL_FORCE'] = False
    

#instantiate graphic utils
os.system("killall rosmaster rviz")
if FLAG.get('SHOW_ANIMATION'):
    ros_pub = RosPub("ur5")
robot = getRobotModel("ur5")


# Init variables
zero = np.array([0.0, 0.0,0.0, 0.0, 0.0, 0.0])
time = 0.0



two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
buffer_size = int(math.floor(conf.exp_duration/conf.dt))
q_log = np.empty((6, buffer_size))*nan
q_des_log = np.empty((6, buffer_size))*nan
qd_log = np.empty((6, buffer_size))*nan
qd_des_log = np.empty((6, buffer_size))*nan
qdd_log = np.empty((6, buffer_size))*nan
qdd_des_log = np.empty((6, buffer_size))*nan
tau_log = np.empty((6, buffer_size))*nan
f_log = np.empty((3,buffer_size))*nan
p_log = np.empty((3, buffer_size))*nan
time_log =  np.empty((buffer_size))*nan
log_counter = 0

# EXERCISE 2.2: 
#conf.qd0 = ...

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)
qd_des_old = zero
q_des_old = conf.q0 

contact_sampled = False

# CONTROL LOOP
while True:
    
    tau = zero
    
    # EXERCISE 1.1: Sinusoidal reference Generation
    if FLAG.get('SIN_WAVE',False):
        q_des   = conf.q0 + conf.amp*np.sin(two_pi_f*time + conf.phi)
        qd_des  = two_pi_f_amp * np.cos(two_pi_f*time + conf.phi);
        qdd_des = - two_pi_f_squared_amp * np.sin(two_pi_f*time + conf.phi);
        if time >= conf.exp_duration_sin:
            q_des = conf.q0
            qd_des= zero
            qdd_des= zero 
    
    # EXERCISE 1.2: Step reference Generation
    if FLAG.get('SQUARE_WAVE',False):
        if time > 2.0:
             q_des = conf.q0 + conf.amp
             qd_des =  zero
             qdd_des = zero
        else:
             q_des = conf.q0
             qd_des =  zero
             qdd_des = zero

 
    # Decimate print of time
    #if (divmod(time ,1.0)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= conf.exp_duration:
        break
                            
    robot.computeAllTerms(q, qd) 
    # joint space inertia matrix                
    M = robot.mass(q)
    # bias terms                
    h = robot.nle(q, qd)
    #gravity terms                
    g = robot.gravity(q)
    
    #compute ee position  in the world frame  
    p = robot.framePlacement(q, frame_ee).translation 
    # compute jacobian of the end effector in the world frame  
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 
        
    # EXERCISE  1.5: PD control - critical damping
    if FLAG.get('CRITICAL_DAMPING',False):
        conf.kd = 2*np.sqrt(conf.kp*M)
                           
    #Exercise 1.3:  PD control
    if FLAG.get('PD_CONTROL',False):       
        tauControl = conf.kp.dot(q_des-q) + conf.kd.dot(qd_des-qd)
        tau = tauControl
    
    # Exercise 1.6: PD control + Gravity Compensation    
    if FLAG.get('GRAVITY_COMPENSATION',False):
        tauG = g
        tau = tauControl + g
    
    # Exercise 1.7: PD + gravity + Feed-Forward term
    if FLAG.get('FEED_FOWARD',False):
        tauFF = M.dot(qdd_des + tauControl)
        tau += tauFF

    			 
    if FLAG.get('EXTERNAL_FORCE',False):   		
        # EXERCISE 2.4: Add external force at T = 3.0s
        if time>3.0:
           F_env = conf.extForce
        else:
           F_env = np.array([0.0, 0.0, 0.0])
        			
        # EXERCISE 2.7: Add  unilateral compliant contact (normal spring)
        # pd = J.dot(qd)
        # if (conf.n.dot(conf.p0 - p)>0.0):
        #     Fk = conf.K_env.dot(conf.p0 - p)
        #     if (conf.n.dot(pd)<0.0):
        #         Fd = - conf.D_env.dot(pd)
        #     F_proj =  np.dot(conf.n, Fk + Fd) #scalar
        #     F_env = conf.n * F_proj #vector
        # else:
        #     F_env = np.array([0.0, 0.0, 0.0])
        #
#       # EXERCISE 2.8: Add  unilateral compliant contact  (full 3D model)
#         pd = J.dot(qd)
#         if (conf.n.dot(conf.p0 - p)>0.0):
#             # sample P0
#             if (not contact_sampled):
#                 conf.p0 = p
#                 contact_sampled = True
#             F_env =  conf.K_env.dot(conf.p0 - p)
#             if (conf.n.dot(pd) < 0.0):
#                 F_env += - conf.D_env.dot(pd)
#
#            # EXERCISE 2.9: Friction coefficient
#             # # clamp to friction cone
#             # # X component
#             # if (F_env[0] >= conf.mu * F_env[2]):
#             #     F_env[0] = conf.mu * F_env[2]
#             # if (F_env[0] <= -conf.mu * F_env[2]):
#             #     F_env[0] = -conf.mu * F_env[2]
#             # # Y component
#             # if (F_env[1] >= conf.mu * F_env[2]):
#             #     F_env[1] = conf.mu * F_env[2]
#             # if (F_env[1] <= -conf.mu * F_env[2]):
#             #     F_env[1] = -conf.mu * F_env[2]
#             # ros_pub.add_cone(p, conf.n, conf.mu, color = "blue")
#
#         else:
#             contact_sampled = False
#             F_env = np.array([0.0, 0.0, 0.0])

        if FLAG.get('SHOW_ANIMATION'):
            ros_pub.add_arrow(p,F_env/100.)
            
        tau += J.transpose().dot(F_env)
    if FLAG.get('SHOW_ANIMATION'):
        ros_pub.add_marker(p)              				
  		      
    #SIMULATION of the forward dynamics    
    M_inv = np.linalg.inv(M)  
    qdd = M_inv.dot(tau-h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt    
    q = q + conf.dt*qd  + 0.5*conf.dt*conf.dt*qdd
    
    # Log Data into a vector
    time_log[log_counter] = time
    q_log[:,log_counter] = q
    q_des_log[:,log_counter] = q_des
    qd_log[:,log_counter] = qd
    qd_des_log[:,log_counter] = qd_des
    qdd_log[:,log_counter] = qdd
    qdd_des_log[:,log_counter] = qdd_des
    p_log[:, log_counter] = p
    tau_log[:,log_counter] = tau
    log_counter+=1               
 
    # update time
    time = time + conf.dt                  
                
    #publish joint variables
    if FLAG.get('SHOW_ANIMATION'):
        ros_pub.publish(robot, q, qd, tau)                   
        tm.sleep(conf.dt*conf.SLOW_FACTOR)
        # stops the while loop if  you prematurely hit CTRL+C                    
        if ros_pub.isShuttingDown():
            print ("Shutting Down")                    
            break;

if FLAG.get('SHOW_ANIMATION'):
    ros_pub.deregister_node()
        
                          
# plot joint variables                                                                              
plotJoint('position', 0, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
plotJoint('velocity', 1, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
plotJoint('acceleration', 2, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
plotJoint('torque', 3, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)

figs = list(map(plt.figure, plt.get_fignums()));
for f in figs:
    f.set_figwidth(10)
    f.set_figheight(5)
    f.savefig('../Relatorio 2/img/fig_%s_%s.png'%(FLAG['POINT'],f.texts[0].get_text()))

#plt.show(block=True)




