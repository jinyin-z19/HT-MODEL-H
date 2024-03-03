#!/usr/bin/python
# -*- coding: UTF-8 -*-
import ctypes

lib = ctypes.CDLL('./walking_packet/MODEL_H_MOTOR_BUILDER/build/lib/libliveltbot_driver_sdk.so')

# Define the function argument and return types
lib.joints_move.argtypes = [ctypes.c_float * 18]
lib.joints_move.restype = None

lib.joints_state.argtypes = [ctypes.c_float * 18]
lib.joints_state.restype = None

class ModelHCtl:
  """Motor control for modle h"""
  def __init__(self):
    """
    initialize class
    """
    # get C list pointer
    self.motor_ang = [0] * 18
    self.motor_rec = [0] * 18
    self.motor_ang_rec = [0] * 18
    self.c_motor_ang = (ctypes.c_float * 18)(*self.motor_ang)
    self.c_motor_rec = (ctypes.c_float * 18)(*self.motor_rec)
    # set pointer
    self.c_motor_ang_ptr = ctypes.pointer(self.c_motor_ang)
    self.c_motor_rec_ptr = ctypes.pointer(self.c_motor_rec)
    # max speed of one command
    self.max_speed = 0.05   


  def MotorMove(self, tar_motor_ang):
    """
    motors move
    """       
    for index in range(18):
      self.c_motor_ang[index] = tar_motor_ang[index]
    lib.joints_move(self.c_motor_ang)


  def MotorCall(self):
    """
    motors pos callback
    """
    lib.joints_state(self.c_motor_rec)       
    theta = ctypes.cast(self.c_motor_rec_ptr, ctypes.POINTER(ctypes.c_float * 18)).contents
    for index in range(18):
      self.motor_ang_rec[index] = theta[index]
    return self.motor_ang_rec

  def MotorSafeMove(self, tar_motor_ang):
    """
    motors move safely
    """ 
    lib.joints_state(self.c_motor_rec)       
    theta = ctypes.cast(self.c_motor_rec_ptr, ctypes.POINTER(ctypes.c_float * 18)).contents    
    for index in range(18):
      if tar_motor_ang[index] - theta[index] > self.max_speed:
        self.c_motor_ang[index] = theta[index] + self.max_speed
      elif tar_motor_ang[index] - theta[index] < -self.max_speed:
        self.c_motor_ang[index] = theta[index] - self.max_speed
      else:
        self.c_motor_ang[index] = tar_motor_ang[index]	
    lib.joints_move(self.c_motor_ang)        

if __name__ == '__main__':
  mctl = ModelHCtl()
  ang_move = mctl.MotorCall()
  print(mctl.MotorCall())
  mctl.MotorMove(ang_move)


