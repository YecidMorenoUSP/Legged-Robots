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
from base_controllers.utils.math_tools import Math
import ex_3_conf as conf

FLAG = {}

FLAG['SHOW_ANIMATION'] = False
FLAG['SIN_WAVE'] = False
FLAG['SQUARE_WAVE'] = False
FLAG['PD_CONTROL'] = False
FLAG['PD_CONTROL_EA'] = False
FLAG['POSTURAL_TASK'] = False
FLAG['GRAVITY_COMPENSATION'] = False
FLAG['FEED_FOWARD'] = False
FLAG['CARTESIAN_ID'] = False
FLAG['CARTESIAN_ID_SIMPLE'] = False
FLAG['LIMS'] = 1
FLAG['POINTS'] = ['1.1','1.2','1.4']
FLAG['POINT'] = '2.2'
    
POINT = FLAG['POINT']
if(POINT == '1.1'):
    FLAG['LIMS'] = 3
    FLAG['SIN_WAVE'] = True
elif(POINT == '1.2'):
    FLAG['SQUARE_WAVE'] = True
    FLAG['LIMS'] = 3
elif(POINT == '1.4'):
    FLAG['PD_CONTROL'] = True
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL_EA'] = True
    FLAG['LIMS'] = 3
elif(POINT == '1.5'):
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL_EA'] = True
    FLAG['POSTURAL_TASK'] = True
    FLAG['LIMS'] = 3
elif(POINT == '1.6'):    
    FLAG['POSTURAL_TASK'] = True
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL_EA'] = True
    FLAG['POSTURAL_TASK'] = True
    FLAG['GRAVITY_COMPENSATION'] = True
    FLAG['LIMS'] = 2
elif(POINT == '1.7'):       
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL_EA'] = True
    FLAG['GRAVITY_COMPENSATION'] = True
    FLAG['POSTURAL_TASK'] = True
    FLAG['FEED_FOWARD'] = True
    FLAG['LIMS'] = 2
elif(POINT == '2.1'):       
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL_EA'] = True
    FLAG['POSTURAL_TASK'] = True
    FLAG['FEED_FOWARD'] = True
    FLAG['CARTESIAN_ID'] = True
    FLAG['LIMS'] = 1
elif(POINT == '2.2'):       
    FLAG['SIN_WAVE'] = True
    FLAG['PD_CONTROL_EA'] = True
    FLAG['POSTURAL_TASK'] = True
    FLAG['FEED_FOWARD'] = True
    FLAG['CARTESIAN_ID_SIMPLE'] = True
    FLAG['LIMS'] = 1
    
    
#instantiate graphic utils
os.system("killall rosmaster rviz")
if FLAG.get('SHOW_ANIMATION'):
    ros_pub = RosPub("ur5")
robot = getRobotModel("ur5")

math_utils = Math()
# Init variables
zero = np.array([0.0, 0.0,0.0, 0.0, 0.0, 0.0])
zero_cart = np.array([ 0.0, 0.0,0.0])
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
buffer_size = int(math.floor(conf.exp_duration/conf.dt))
log_counter = 0
p_log = np.empty((3, buffer_size))*nan
p_des_log = np.empty((3,buffer_size))*nan
pd_log = np.empty((3,buffer_size))*nan
pd_des_log = np.empty((3,buffer_size))*nan
pdd_des_log = np.empty((3,buffer_size))*nan
rpy_log = np.empty((3,buffer_size))*nan
rpy_des_log = np.empty((3,buffer_size))*nan
error_o_log = np.empty((3,buffer_size))*nan
tau_log = np.empty((6,buffer_size))*nan
time_log =  np.empty((buffer_size))*nan

rpy_old = np.zeros((3))
rpy_unwrapped = np.zeros((3))
rpy_des_old = np.zeros((3))
rpy_des_unwrapped = np.zeros((3))


q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

# compute initial end effector position and velocity from q0
p0 = robot.framePlacement(conf.q0, frame_ee, True).translation + np.array([0.0, 0.0, 0.0])
pd0 = np.array([ 0.0, 0.0, 0.0])
pdd0 = np.array([ 0.0, 0.0, 0.0])

p = p0
pd = pd0
pdd = pdd0
p_des = p0
pd_des = zero_cart
pdd_des = zero_cart
rpy = zero_cart
rpy_des = zero_cart
FirstTime = True

# CONTROL LOOP
while True:
    
    
    # EXERCISE 1.1: Sinusoidal reference generation for the end effector   
    if FLAG.get('SIN_WAVE',False):
        p_des   = p0 + conf.amp*np.sin(two_pi_f*time + conf.phi)
        pd_des  = two_pi_f_amp * np.cos(two_pi_f*time + conf.phi);
        pdd_des = - two_pi_f_squared_amp * np.sin(two_pi_f*time + conf.phi);
        if time >= conf.exp_duration_sin:
            p_des = p0
            pd_des= pd0
            pdd_des= pdd0 

    #  EXERCISE 1.2: Step reference generation for the end effector 
    if FLAG.get('SQUARE_WAVE',False):
        if time > 2.0:
            p_des = p0 + conf.amp
            qd_des =  pd0
            qdd_des = pdd0
        else:
            p_des = p0
            pd_des = pd0
            pdd_des= pdd0 

        


#    EXERCISE 2.3: Constant reference


    # Decimate print of time
    #if (divmod(time ,1.0)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= conf.exp_duration:
        break
                            
    robot.computeAllTerms(q, qd) 
    # joint space inertia matrix                
    M = robot.mass(q, False)
    # bias terms                
    h = robot.nle(q, qd, False)
    #gravity terms                
    g = robot.gravity(q)
    
    # compute jacobian of the end effector in the world frame    
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 
    # compute  the end-effector acceleration due to joint velocity Jdot*qd         
    Jdqd = robot.frameClassicAcceleration(q, qd, None, frame_ee).linear    
    # compute frame end effector position and velocity in the WF   
    p = robot.framePlacement(q, frame_ee).translation  

    # with sine reference: to avoid having tracking errors in velocity at the initial point
#    if FirstTime:    
#        qd = J.T.dot(np.linalg.inv(J.dot(J.T))).dot(two_pi_f_amp)
#        FirstTime = False
                    
    pd = J.dot(qd)  


    M_inv = np.linalg.inv(M)     
    # Moore-penrose pseudoinverse  A^# = (A^TA)^-1 * A^T with A = J^T
    JTpinv = np.linalg.inv(J.dot(J.T)).dot(J)
    
    # joint space inertia matrix reflected at the end effector (J*M^-1*Jt)^-1
    lambda_= np.linalg.inv(J.dot(M_inv).dot(J.T))  # J should be full row rank  otherwise add a damping term
     
    #Null space projector I - (JTpinv )^-1 * JTpinv => I  - JT *JTpiv
    N = eye(6)-J.T.dot(JTpinv)
    
    tau = 0    
    
    if FLAG.get('PD_CONTROL',False) :
        None
    
    # EXERCISE 1.4: PD control (cartesian task)  
    tauControl = 0
    tau_EA = 0
    if FLAG.get('PD_CONTROL_EA',False) :
        tauControl =  conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des - pd) 
        tau_EA = J.T.dot( tauControl )
        tau += tau_EA
      
    tau_null = 0
    # EXERCISE 1.5: PD control (cartesian task) + postural task  
    if FLAG.get('POSTURAL_TASK',False) :
        tau_0 = conf.Kq*(conf.q0-q) - conf.Dq*qd
        tau_null = N.dot(tau_0)
        tau += tau_null
    
    tauGnull = 0
    # EXERCISE 1.6: PD control + Gravity Compensation:    
    if FLAG.get('GRAVITY_COMPENSATION',False) :
        tauGnull = J.T.dot(JTpinv.dot(g))
        tau += tauGnull
    
    tau_ff = 0
    # EXERCISE 1.7: PD control  + Gravity Compensation + Feed-Forward term
    if FLAG.get('FEED_FOWARD',False) :
        tau_ff = J.T.dot(lambda_.dot(pdd_des))
        tau += tau_ff 
    
    tau_ID = 0
    # EXERCISE 2.1: Cartesian space inverse dynamics
    if FLAG.get('CARTESIAN_ID',False) :  
        Fdes = pdd_des + tauControl
        mu =  JTpinv.dot(h)-lambda_.dot(Jdqd)
        tau_ID = J.T.dot( lambda_.dot(Fdes) + mu) + tau_null
        tau = tau_ID
    
    tau_ID_S = 0
    # EXERCISE 2.2: Cartesian space inverse dynamics with bias compensation in joint space (simpler to compute)
    if FLAG.get('CARTESIAN_ID_SIMPLE',False) :      
        Fdes = pdd_des + tauControl
        tau_ID_S = J.T.dot( lambda_.dot(Fdes)) + tau_null + h
        tau = tau_ID_S


     # EXERCISE 3.1:  Control of orientation with PD: constant orientation

    
    #compose the des orientation rotation matrix 180deg about x (x axis along x, y axis along -y, z axis along -z) NB the axis are the columns of the matrix w_R_des 


    # EXERCISE 3.2 - Control of orientation with PD: singularity with Euler Angles


    # EXERCISE 3.3: Control of orientation with PD - sinusoidal reference 

   
   
    # EXERCISE 3.1: Control of orientation with PD   
#    # compute rotation matrix from actual orientation of ee to the desired
#    e_R_des = w_R_e.T.dot(w_R_des)    
#    # compute the angle-axis representation of the associated orientation error				   
    # compute the angle: method 1) with arc cos
#    cos_theta = (e_R_des[0,0]+ e_R_des[1,1]+ e_R_des[2,2]-1)/2
#    delta_theta = np.arccos( cos_theta) 
    # compute the angle: method 2) with atan2
#    delta_theta = math.atan2(np.sqrt(pow(e_R_des[2,1]-e_R_des[1,2], 2) +  pow(e_R_des[0,2]-e_R_des[2,0], 2) + pow(e_R_des[1,0]-e_R_des[0,1], 2)), e_R_des[0,0]+ e_R_des[1,1]+ e_R_des[2,2]-1 )  
#    # compute the axis    
#    r_hat = 1/(2*np.sin(delta_theta))*np.array([e_R_des[2,1]-e_R_des[1,2], e_R_des[0,2]-e_R_des[2,0], e_R_des[1,0]-e_R_des[0,1]])     
#    # compute the orientation error
#    e_error_o = delta_theta * r_hat 
#    # the error is expressed in the end-effector frame 
#    # we need to map it in the world frame to compute the moment because the jacobian is in the WF
#    w_error_o = w_R_e.dot(e_error_o) 				
#    # Compute the virtual force (linear part of the wrench) 
#    F_des = conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)
#    # compute the virtual moment (angular part of the wrench) to realize the orientation task
#    Gamma_des =  conf.Ko.dot(w_error_o) + conf.Do.dot(omega_des - omega)
#    # stack the previously computed linear part with the angular part
#    W_des = np.hstack([F_des, Gamma_des])
#    # map to torques
#    tau = J6.T.dot(W_des)  + g          
#    # extract actual euler angles for logging   
#    rpy = math_utils.rot2eul(w_R_e)



    # EXERCISE 3.4: Control of orientation with PD - unwrapping  
    #unwrap euler angles   
#    UNWRAPPPING = True
#    for i in range(3):
#        rpy_unwrapped[i] = rpy[i];
#        while (rpy_unwrapped[i] < rpy_old[i]  - math.pi):
#            rpy_unwrapped[i] += 2*math.pi
#        while (rpy_unwrapped[i] > rpy_old[i]  + math.pi):
#            rpy_unwrapped[i] -= 2*math.pi
#        rpy_old[i] = rpy_unwrapped[i]
#    for i in range(3):
#        rpy_des_unwrapped[i] = rpy_des[i];
#        while (rpy_des_unwrapped[i] < rpy_des_old[i]  - math.pi):
#            rpy_des_unwrapped[i] += 2*math.pi
#        while (rpy_des_unwrapped[i] > rpy_des_old[i]  + math.pi):
#            rpy_des_unwrapped[i] -= 2*math.pi
#        rpy_des_old[i] = rpy_des_unwrapped[i]
    
    #EXERSISE 3.6 : full task space inverse dynamics (computed torque)
    # compute lambda for both orientation and position
    rho = 0.00001 # damping factor
#    print "J6 rank:", np.linalg.matrix_rank(J6)
#    lambda6_= np.linalg.inv(J6.dot(M_inv).dot( J6.T))  #singular
#    lambda6_ = np.linalg.inv(J6.dot(M_inv).dot( J6.T) + pow(rho,2)*eye(6)) # damped inertia matrix
#    #J6Tpinv = np.linalg.pinv(J6.T, rho) # damped pseudoinverse using native function     
#    J6Tpinv = np.linalg.inv(J6.dot(J6.T) + pow(rho,2)*eye(6)).dot(J6)  # damped pseudoinverse explicitely computed  
#    Jdqd6 = robot.frameClassicAcceleration(q, qd, None, frame_ee).vector   
#    mu6 =  J6Tpinv.dot(h) -lambda6_.dot(Jdqd6)      
#    tau = J6.T.dot(lambda6_.dot(np.hstack((pdd_des + F_des, omega_d_des + Gamma_des))) + mu6) 
    			
#    EXERCISE 2.3: Add an external force
    if conf.EXTERNAL_FORCE  and time>1.0:
        tau += J.transpose().dot(conf.extForce)
        ros_pub.add_arrow(p, conf.extForce/100)                    
    
    #SIMULATION of the forward dynamics    
    qdd = M_inv.dot(tau - h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt
    q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd
				    
    # Log Data into a vector
    time_log[log_counter] = time
    p_log[:,log_counter] = p
    p_des_log[:,log_counter] = p_des
    pd_log[:,log_counter] = pd
    pd_des_log[:,log_counter] = pd_des
    tau_log[:,log_counter] = tau
  
    try: 
        UNWRAPPPING
        rpy_log[:,log_counter] = rpy_unwrapped
        rpy_des_log[:,log_counter] = rpy_des_unwrapped
    except:    
        rpy_log[:,log_counter] = rpy
        rpy_des_log[:,log_counter] = rpy_des
    try: 
        ORIENTATION_CONTROL
        error_o_log[:,log_counter] =  w_error_o
    except: 
        pass                      
 
    log_counter+=1                     
 
    # update time
    time = time + conf.dt                  
    
    
    if FLAG.get('SHOW_ANIMATION'):
        # plot ball at the end-effector
        ros_pub.add_marker(p)                   
        #publish joint variables
        ros_pub.publish(robot, q, qd, tau)                   
        tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if FLAG.get('SHOW_ANIMATION'):
        if ros_pub.isShuttingDown():
            print ("Shutting Down")                    
            break;            
if FLAG.get('SHOW_ANIMATION'):
    ros_pub.deregister_node()
      
#plot position
plotEndeff('position', 1,time_log, p_log, p_des_log)
plotEndeff('velocity', 2, time_log, p_log, p_des_log, pd_log, pd_des_log, rpy_log, rpy_des_log)
try:
    ORIENTATION_CONTROL
    plotEndeff('orientation', 3,time_log, p_log, p_des_log, pd_log, pd_des_log, rpy_log, rpy_des_log)
    plotEndeff('orientation', 4,time_log, p_log, p_des_log, pd_log, pd_des_log, error_o_log)
except: 
    pass   

if FLAG.get('LIMS',1) == 1:
    ylims = (p0[0]+np.array([-1,1])*conf.amp[0]*1.3,
             p0[1]+np.array([-1,1])*.0003,
             p0[2]+np.array([-1,.4])*.002)
elif FLAG.get('LIMS',1) == 2:
    ylims = (p0[0]+np.array([-1,1])*conf.amp[0]*1.3,
             p0[1]+np.array([-1,1])*.005,
             p0[2]+np.array([-1,1])*.008)
elif FLAG.get('LIMS',1) == 3:
    ylims = (p0[0]+np.array([-1,1])*conf.amp[0]*1.5,
             p0[1]+np.array([-1,1])*.025,
             p0[2]+np.array([-1,.2])*.15)

figs = list(map(plt.figure, plt.get_fignums()));
for idx,f in enumerate(figs):
    ax = f.get_axes()
    if(idx == 0):
        ax[0].set_ylim(ylims[0])
        ax[1].set_ylim(ylims[1])
        ax[2].set_ylim(ylims[2])
    f.set_figwidth(10)
    f.set_figheight(5)
    if(POINT == '1.1') | (POINT == '1.2'):
        figs[0].set_figwidth(5)
        figs[0].set_figheight(5)
    f.savefig('../Relatorio 3/img/fig_%s_%s.png'%(FLAG['POINT'],f.texts[0].get_text()))
#plt.show(block=True)

    