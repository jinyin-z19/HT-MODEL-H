#!/usr/bin/python
# -*- coding: UTF-8 -*-

import math
import numpy as np
import time
import ctypes
lib = ctypes.CDLL('./build/libthmos_leg_ik.so')

# Define the function argument and return types
lib.leg_ik.argtypes = [
    ctypes.c_double * 3,
    ctypes.c_double * 6,
    ctypes.c_double * 6,
    ctypes.c_double * 6,
]
lib.leg_ik.restype = None


class LegIK:
    """inverse kinematics for one leg"""
    def __init__(self, LeftorRight, Legs_len, motor_offset, motor_way):
        """
        initialize class
        Args:
            LeftorRight :id-left leg or right leg string-'left' 'right' 
            Legs_len : a list of leg len [3]
            motor_offset : a list of offset angle [6]
            motor_way : a list of spin direction of motor [6,+1,-1]
        """
        self.LorR = LeftorRight
        self.legs_len = Legs_len
        self.offset = motor_offset
        self.way = motor_way
        self.theta = [0] * 6

    def RtoRPY(self, R):
        '''rotate matrix to RPY angles'''
        R = np.array(R)
        err = float(0.001)
        oy = math.atan2(-R[2,0], math.sqrt((R[0,0])**2 + (R[1,0])**2))

        if oy >= math.pi/2-err and oy <= math.pi/2+err:
            oy = math.pi/2
            oz = 0.0
            ox = math.atan2(R[0,1], R[0,2])
        elif oy >= -(math.pi/2)-err and oy <= -(math.pi/2)+err:
            oy = -math.pi/2
            oz = 0.0
            ox = math.atan2(-R[0,1], -R[0,2])
        else:
            oz = math.atan2((R[1,0])/(math.cos(oy)), (R[0,0])/(math.cos(oy)))
            ox = math.atan2((R[2,1])/(math.cos(oy)), (R[2,2])/(math.cos(oy)))

        return [ox, oy, oz]

    def RPYtoR(self,rpy):
        '''RPY angles to rotate matrix'''
        a = rpy[0]
        b = rpy[1]
        c = rpy[2]

        sinA = np.sin(a)
        cosA = np.cos(a)
        sinB = np.sin(b)
        cosB = np.cos(b)
        sinC = np.sin(c)
        cosC = np.cos(c)

        R = [[cosB*cosC,  cosC*sinA*sinB - cosA*sinC,  sinA*sinC + cosA*cosC*sinB],
             [cosB*sinC,  cosA*cosC + sinA*sinB*sinC, cosA*sinB*sinC - cosC*sinA],
             [-sinB, cosB*sinA,  cosA*cosB]]
        return R

    def axistoR(self, axis, theta):
        '''torch to transform [numpy.array]'''
        if(axis == 1):
          #axis x
          R = np.mat([[ 1            , 0            , 0            ],
                      [ 0            , np.cos(theta),-np.sin(theta)],
                      [ 0            , np.sin(theta), np.cos(theta)]])
        elif(axis == 2):
          #axis y
          R = np.mat([[ np.cos(theta), 0            , np.sin(theta)],
                      [ 0            , 1            , 0            ],
                      [-np.sin(theta), 0            , np.cos(theta)]])
        else:
          #axis z
          R = np.mat([[ np.cos(theta),-np.sin(theta), 0            ],
                      [ np.sin(theta), np.cos(theta), 0            ],
                      [ 0            , 0            , 1            ]])        
        return R

    def StdLegIK(self, end_point, end_rpy):
        '''inverse kinematics with standard axis'''
        
        #get foot target transformation
        Rt = np.array(self.RPYtoR(end_rpy))
        dt = np.array(end_point)

        #get inverse to change coordinates(from lab to foot)
        Rti = Rt.T
        d = np.dot(Rti, -dt)

        #caculate self.theta 3 4 5
        la = np.linalg.norm(d - np.array([0,0,self.legs_len[2]]))

        # 3
        if(abs(self.legs_len[0] - self.legs_len[1]) > la):
            self.theta[3] = - np.pi 
            theta_a = np.pi
        elif(self.legs_len[0] + self.legs_len[1] > la):
            self.theta[3] = np.arccos( (self.legs_len[0] ** 2  + self.legs_len[1] ** 2 - la ** 2)  / (2 * self.legs_len[0] * self.legs_len[1]) )- np.pi
            theta_a  = np.arccos( (self.legs_len[1] ** 2  + la ** 2 - self.legs_len[0] ** 2) / (2 * self.legs_len[1] * la) )
        else:
            self.theta[3] = 0
            theta_a = 0
        
        # 4
        self.theta[4] = theta_a + np.arcsin(d[0] / la)
        
        # 5
        self.theta[5] = np.arctan(- d[1] / (d[2] - self.legs_len[2]))

        #caculate self.theta 0 1 2
        R5 = np.mat(self.axistoR(1,self.theta[5]))
        R43 = np.mat(self.axistoR(2,self.theta[4] + self.theta[3]))
        Rfoot = R5 * R43
        Rlap = Rfoot.T * np.mat(Rti)
        self.theta[0:3] = self.RtoRPY(Rlap)
        
        #set theta
        for index in range(6):
            self.theta[index] = self.theta[index] * self.way[index] + self.offset[index] 
        return self.theta

class THMOSLegIK:
    """inverse kinematics for two leg"""
    def __init__(self, way_left = [1.0,-1.0,-1.0,-1.0,-1.0,-1.0], way_right = [1.0,1.0,-1.0,1.0,1.0,-1.0],leg_rod_length = [0.156,0.12,0.045]):
        """
        initialize class
        """
        self.leg_left = LegIK('Left',leg_rod_length, [0,0,0,0,0,0], way_left)
        self.leg_right = LegIK('Right',leg_rod_length, [0,0,0,0,0,0],way_right)
        self.leg_rod_length = leg_rod_length
        self.way_left = way_left
        self.way_right = way_right
        # get C list pointer
        self.leg_ang = [0] * 6
        self.end_pos = [0] * 6
        self.c_leg_ang = (ctypes.c_double * 6)(*self.leg_ang)
        self.c_end_pos = (ctypes.c_double * 6)(*self.end_pos)
        self.c_leg_len = (ctypes.c_double * 3)(*self.leg_rod_length)
        self.c_motor_way_l = (ctypes.c_double * 6)(*self.way_left)
        self.c_motor_way_r = (ctypes.c_double * 6)(*self.way_right)
        # set pointer
        self.c_leg_ang_ptr = ctypes.pointer(self.c_leg_ang)
        self.c_end_pos_ptr = ctypes.pointer(self.c_end_pos)
        self.c_leg_len_ptr = ctypes.pointer(self.c_leg_len)
        self.c_motor_way_l_ptr = ctypes.pointer(self.c_motor_way_l)
        self.c_motor_way_r_ptr = ctypes.pointer(self.c_motor_way_r)    
            
    def LegIKMove(self, LeftorRight, end_pos, body_rpy = [0,0,0]):
        """
        move left and right leg
        """
        #sst = time.time()
        
        xyz = end_pos[0:3]
        rpy = end_pos[3:6]
        
        if (LeftorRight == 'Left' or LeftorRight == 'left'):
            self.leg_ang = self.leg_left.StdLegIK(xyz, rpy)
        else:
            self.leg_ang = self.leg_right.StdLegIK(xyz, rpy)
        #eed = time.time()
        #print('mid', sst - eed) 
        return self.leg_ang
 
    def LegIKMoveCpp(self, LeftorRight, end_pos):
        """
        move left and right leg
        """
        #sst = time.time()
        
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
        #eed = time.time()
        
        #print('mid', sst - eed)        
        return self.leg_ang
        
               
if __name__ == '__main__':
    leg_1 = THMOSLegIK()
    st = time.time()
    a = leg_1.LegIKMoveCpp("left",[0, 0.01, -0.32, 0.0, 0.01, 0.0])
    ed1 = time.time() 
    b = leg_1.LegIKMove("left",[0, 0.01, -0.32, 0.0, 0.01, 0.0])
    ed2 = time.time()
    print(ed1 - st, ed2 -ed1)
    print('cpp',a)
    print('py',b)
