# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

LINE_WIDTH = 40

trajectory_name = "solo_trajectory_trot"

dt = 0.004  # controller time step
exp_duration = 5.0 #simulation duration
CONTINUOUS = False
verbose = True

ee_names = ["LF","RF","LH","RH"]

buffer_size = 1501

# Gains for the joint PD controller
kp = 6.0  # 30.0
ki = 0.0   # 0.0
kd = 0.2   # 1.0

# Gains for the motion tracking control
Kp_base_rot = 100.0
Kd_base_rot = 20.0
Kp_base_trans = 100.0
Kd_base_trans = 20.0
Kp_swingfeet = 200.0
Kd_swingfeet = 30.0

