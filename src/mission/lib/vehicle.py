#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 24 23:34:52 2018

@author: vijayaganesh
"""

import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped

from mavros.srv import CommandBool, SetMode

from lib.position import Positions

class Vehicle(object,Positions):
  """
  Class for arming and Prearm flight Checks
  """
  
  def __init__(self):
    Positions.__init__(self)
    self.setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local_position", PoseStamped, queue_size=10)
    self.setvel_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size = 10)
    self.setaccel_publisher = rospy.Publisher("/mavros/setpoint_accel/accel",Vector3Stamped,queue_size=10)
  
  
  
  
  def arm(self):
    pass
  
  def disarm(self):
    pass
  
  def command_arm(self):
    rospy.Wat