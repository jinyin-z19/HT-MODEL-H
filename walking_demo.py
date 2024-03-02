#!/usr/bin/env python3
import numpy as np
import sys
sys.path.append('./walking_packet')
from thmos_walk_engine import *
from random import random 
from time import sleep
import time

if __name__ == '__main__':
  TIME_STEP = 0.001
  # initial ---
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
          walk.setGoalVel([(random()-0.5)*0.0, (random()-0.5)*0.0 + 0.05, (random()-0.5)*0.0])
          nk = nk + 1
        elif nk < 12:
          walk.setGoalVel([(random()-0.5)*0.0, (random()-0.5)*0.0, (random()-0.5)*0.0])
          nk = nk + 1
        else:
          walk.setGoalVel([(random()-0.5)*0.0, (random()-0.5)*0.0, (random()-0.5)*0.0])
          nk = 0
      joint_angles,n = walk.getNextPos()
      j = 0
    
      # simulation / motor control ---
      # print(joint_angles)

