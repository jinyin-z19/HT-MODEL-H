#!/usr/bin/python
# -*- coding: UTF-8 -*-

import math
import numpy as np
import time
import ctypes
lib = ctypes.CDLL('./build/libmodel_h_leg_ik.so')

# Define the function argument and return types
lib.leg_ik.argtypes = [
    ctypes.c_double * 7,
    ctypes.c_double * 6,
    ctypes.c_double * 6,
    ctypes.c_double * 6,
]
lib.leg_ik.restype = None

class ModelHLegIK:
    """inverse kinematics for two leg"""
    def __init__(self, way_left = [1.0,-1.0,1.0,-1.0,-1.0,-1.0], way_right = [-1.0,1.0,-1.0,1.0,-1.0,-1.0],leg_rod_length = [0.15,0.16,0.0255,0.03215,0.022,0.112,0.065]):
        """
        initialize class
        """
        self.leg_rod_length = leg_rod_length
        self.way_left = way_left
        self.way_right = way_right
        # get C list pointer
        self.leg_ang = [0] * 6
        self.end_pos = [0] * 6
        self.c_leg_ang = (ctypes.c_double * 6)(*self.leg_ang)
        self.c_end_pos = (ctypes.c_double * 6)(*self.end_pos)
        self.c_leg_len = (ctypes.c_double * 7)(*self.leg_rod_length)
        self.c_motor_way_l = (ctypes.c_double * 6)(*self.way_left)
        self.c_motor_way_r = (ctypes.c_double * 6)(*self.way_right)
        # set pointer
        self.c_leg_ang_ptr = ctypes.pointer(self.c_leg_ang)
        self.c_end_pos_ptr = ctypes.pointer(self.c_end_pos)
        self.c_leg_len_ptr = ctypes.pointer(self.c_leg_len)
        self.c_motor_way_l_ptr = ctypes.pointer(self.c_motor_way_l)
        self.c_motor_way_r_ptr = ctypes.pointer(self.c_motor_way_r)    
            
 
    def LegIKMove(self, LeftorRight, end_pos):
        """
        move left and right leg
        """       
        for index in range(6):
            self.c_end_pos[index] = end_pos[index]
        # caculate 
        if (LeftorRight == 'Left' or LeftorRight == 'left'):
            lib.leg_ik(self.c_leg_len, self.c_end_pos, self.c_motor_way_l, self.c_leg_ang)
        else:
            lib.leg_ik(self.c_leg_len, self.c_end_pos, self.c_motor_way_r, self.c_leg_ang)
        theta = ctypes.cast(self.c_leg_ang_ptr, ctypes.POINTER(ctypes.c_double * 6)).contents
        for index in range(6):
            self.leg_ang[index] = theta[index]
        return self.leg_ang
        
               
if __name__ == '__main__':
    leg_1 = ModelHLegIK()
    a = leg_1.LegIKMove("left",[-0.3355, 0, 0, 0 , -np.pi/2 , 0])
    print(np.array(a))
    a = leg_1.LegIKMove("left",[0, -0.3355, 0, -np.pi/2, 0, 0])
    print(np.array(a))
    a = leg_1.LegIKMove("left",[0, 0, -0.3355, 0, 0, -np.pi/2])
    print(np.array(a))
    a = leg_1.LegIKMove("left",[0, 0.08, -0.28, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, -0.08, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, 0.0001, -0.31, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, -0.0001, -0.31, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, 0, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, -0, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    print("right ----")
    a = leg_1.LegIKMove("right",[-0.3355, 0, 0, 0 , -np.pi/2 , 0])
    print(np.array(a))
    a = leg_1.LegIKMove("right",[0, -0.3355, 0, -np.pi/2, 0, 0])
    print(np.array(a))
    a = leg_1.LegIKMove("right",[0, 0, -0.3355, 0, 0, -np.pi/2])
    print(np.array(a))
    a = leg_1.LegIKMove("right",[0, 0.08, -0.28, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, -0.08, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, 0.0001, -0.31, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, -0.0001, -0.31, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, 0, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, -0, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
