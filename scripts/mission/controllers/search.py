#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 14:55:35 2018

@author: vijayaganesh
"""
import numpy as np
import rospy
from utilities import Utils

from controller import Controller

# Maximum Search speed, in m/s
DEFAULT_MAX_SPEED_XY = 1



class Search_Controller(Controller):
  def __init__(self, *args, **kwargs):
    self.NAME = "Search Controller"
    super(Search_Controller,self).__init__(*args, **kwargs)
    self.l = rospy.get_param("~major_axis")
    self.w = rospy.get_param("~minor_axis")
    self.x0 = rospy.get_param("~x_center")
    self.y0 = rospy.get_param("~y_center")
    self.alt = rospy.get_param("~alt")
    self.grid_step = rospy.get_param("~scan_step")
    self.search_speed = rospy.get_param("~search_speed",DEFAULT_MAX_SPEED_XY)
  
  
  def enter(self):
    self.drone.set_xy_speed(self.search_speed)
    rospy.loginfo("Entering Search Mode")
    self.wp_list = Utils.elliptical_wp(l,w,x0,y0,grid_step,alt)
    
  def handle_track_message(self,msg):
    self.target_position = msg.track.position
    self.
    
    