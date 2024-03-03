#!/usr/bin/python
# -*- coding: UTF-8 -*-

import numpy as np
import sys
sys.path.append('./walking_packet')
from thmos_walk_engine import *
from model_h_motors_control import *
from random import random 
from time import sleep
import time

if __name__ == '__main__':
  TIME_STEP = 0.001
  # initial ---
  mctl = ModelHCtl()
  arm_r_ang = [0, -0.1, 0]
  arm_l_ang = [0, 0.1, 0]
  # -- stand leg zero position --    
  zero_list = [0] * 18
    
  # right leg
  zero_list[0] = 0.45
  zero_list[1] =-0.05
  zero_list[2] = 0.00
  zero_list[3] = 0.08
  zero_list[4] = 0.00
  zero_list[5] = 0.00
    
  # right arm
  zero_list[6] = 0.00
  zero_list[7] = -0.1
  zero_list[8] = 0.00
        
  # left leg
  zero_list[9] = -0.25
  zero_list[10] = 0.05
  zero_list[11] =-0.03
  zero_list[12] =-0.08
  zero_list[13] = 0.00
  zero_list[14] = 0.00
    
  # left arm
  zero_list[15] = 0.00
  zero_list[16] = 0.10
  zero_list[17] = 0.00
  
  # control box ----
  sys.path.append(sys.path[0] + '/param.txt')
  param_path=sys.path[-1]		
  param=np.genfromtxt(fname=param_path,dtype=float,delimiter=",",comments="#",max_rows=38,invalid_raise=False)
  Params = {              
            'foot_width' : param[0],
            'ex_foot_width' : param[1],
            'foot_height' :param[2],
            'com_height' : param[3],
            'com_x_offset' : param[4],
            'com_y_offset' :param[5],
            'trunk_height' : param[6],
            'walking_period' : param[7],
            'both_foot_support_time' : param[8],
            'dt' : param[9],
            'max_vx' : param[10],
            'max_vy': param[11],
            'max_vth' : param[12],
            'k_x_offset':param[13],#ex_com_x_offset k
            'k_y_offset':param[14],#ex_com_y_offset k
            'trunk_pitch':param[15],
            'way_left' : [1.0,-1.0,1.0,-1.0,-1.0,-1.0],
            'way_right' : [-1.0,1.0,-1.0,1.0,-1.0,-1.0],
            'leg_rod_length' : [0.15,0.16,0.0255,0.03215,0.022,0.112,0.065]
            }

  walk = walking(**Params)
  j = 0
  n = 0
  k = 0 
  nk = 0

  while (1):
    j += 1
    if j >= 10:   
      if n == 0:
        if nk < 8:
          walk.setGoalVel([(random()-0.5)*0+0.05, (random()-0.5)*0.0, (random()-0.5)*0.0])
          nk = nk + 1
        elif nk < 12:
          walk.setGoalVel([(random()-0.5)*0.0, (random()-0.5)*0.0, (random()-0.5)*0.0])
          nk = nk + 1
        else:
          walk.setGoalVel([(random()-0.5)*0.0, (random()-0.5)*0.0, (random()-0.5)*0.0])
          nk = 0
      leg_r_ang,leg_l_ang,n = walk.getNextPos()
      j = 0
    
      # simulation / motor control ---
      command_list = leg_r_ang + arm_r_ang + leg_l_ang + arm_l_ang
      ang_move = np.array(command_list)+ np.array(zero_list)
      # print(ang_move)
      mctl.MotorSafeMove(ang_move.tolist())
      sleep(0.03)

