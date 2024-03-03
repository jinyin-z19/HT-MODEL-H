#!/usr/bin/python
# -*- coding: UTF-8 -*-
import sys

print("Python version:", sys.version)

import numpy as np

sys.path.append("./walking_packet")
from model_h_kinematics import *
from model_h_motors_control import *
from time import sleep
import time

if __name__ == "__main__":
    leg_rc = ModelHLegIK()
    leg_lc = ModelHLegIK()
    leg_r_ang = leg_rc.LegIKMove('right', [0, 0, -0.30, 0, 0, 0]).copy()
    arm_r_ang = [0, -0.1, 0]
    leg_l_ang = leg_lc.LegIKMove('left', [0.05, 0, -0.30, 0, 0, 0]).copy()
    arm_l_ang = [0, 0.1, 0]
    command_list = leg_r_ang + arm_r_ang + leg_l_ang + arm_l_ang
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
    ang_move = np.array(command_list)+ np.array(zero_list)
    print(ang_move)
    mctl = ModelHCtl()
    for i in range(100):
      mctl.MotorSafeMove(ang_move.tolist())
      sleep(0.05)
