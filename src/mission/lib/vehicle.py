#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 24 23:34:52 2018

@author: vijayaganesh
"""

import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped,Vector3Stamped

from mavros.srv import CommandBool, SetMode

from mission.lib.position import Positions

class Vehicle(object,Positions):
  """
  Class for arming and Prearm flight Checks
  
  TODO: Add Codebase to perform prearm checks
  """
  
  def __init__(self):
    Positions.__init__(self)
    self.panic = False #Panic variable as failsafe
    self.pass_prearm = True #TODO Alter this parameter after adding the codebase for prearm check
    self.setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local_position", PoseStamped, queue_size=10)
    self.setvel_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size = 10)
    self.setaccel_publisher = rospy.Publisher("/mavros/setpoint_accel/accel",Vector3Stamped,queue_size=10)
  
  def command_mode(self,mode='OFFBOARD'):
    rospy.wait_for_service('/mavros/set_mode')
    command_service = rospy.ServiceProxy('/mavros/set_mode',SetMode)
    num = 0 if mode == 'OFFBOARD' else 1
    command_service(num,mode)
    
    
  def setpoint_pose(self,set_pose):    
    self.setpoint_publisher.publish(set_pose)
  
  def setpoint_vel(self,set_vel):
    self.setvel_publisher.publish(set_vel)
    
  def setpoint_accel(self,set_accel):
    self.setaccel_publisher.publish(set_accel)
  
  def arm(self):
    try:
      assert self.pass_prearm
    except AssertionError:
      print("Pre-Arm Failed! Going to Panic Mode")
      self.panic = True
      return
    self.command_arm(True)
    
      
  def disarm(self):
    """
      TODO: Add code to check whether the drone has landed or in safe altitude before disarming
    """    
    self.command_arm(False)
  
  def command_arm(self,arm):
    rospy.wait_for_service('/mavros/cmd/arming')
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    arm_service(arm)