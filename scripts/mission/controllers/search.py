#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 14:55:35 2018

@author: vijayaganesh
"""

import rospy
from utilities import Utils
from geometry_msgs.msg import PoseStamped

from controller import Controller
from approach import Approach_Controller

# Maximum Search speed, in m/s
DEFAULT_MAX_SPEED_XY = 5



class Search_Controller(Controller):
  def __init__(self, *args, **kwargs):
    self.NAME = "Search Controller"
    super(Search_Controller,self).__init__(*args, **kwargs)
    l = rospy.get_param("~major_axis")
    w = rospy.get_param("~minor_axis")
    x0 = rospy.get_param("~x_center")
    y0 = rospy.get_param("~y_center")
    alt = rospy.get_param("~alt")
    grid_step = rospy.get_param("~scan_step")
    self.search_speed = rospy.get_param("~search_speed",DEFAULT_MAX_SPEED_XY)
#    self.wp_list = Utils.elliptical_wp(l,w,x0,y0,grid_step,alt)
    self.wp_list = Utils.single_wp()
    self.track_counter = 0
  
  def enter(self):
    self.drone.set_xy_speed(self.search_speed)
    rospy.loginfo("Entering Search Mode")
    self.drone.command_offboard()
    self.drone.arm()
    pose = PoseStamped()
    curr_sp = self.wp_list.pop()
    pose.pose.position.x = curr_sp[0]
    pose.pose.position.y = curr_sp[1]
    pose.pose.position.z = curr_sp[2]
    self.curr_sp = pose
    
    
  def handle_track_message(self,msg):
    self.tracking = msg.track.tracking.data
    if self.tracking:
      self.track_counter += 1
    if self.track_counter > 5:
      print("Changing to approach Mode")
      self.mission.switch_state(Approach_Controller(self.mission,self.drone))
    
    
  def run(self):
      if abs(Utils.compute_distance(self.curr_sp,self.drone.pose)) > 0.5:
          self.drone.setpoint_pose(self.curr_sp)
      else:
        if not len(self.wp_list) == 0:
          curr_sp = self.wp_list.pop()
          self.curr_sp.pose.position.x = curr_sp[0]
          self.curr_sp.pose.position.y = curr_sp[1]
          self.curr_sp.pose.position.z = curr_sp[2]
#        else:
#          self.mission.switch_state(Search_Controller(self.mission,self.drone))
    