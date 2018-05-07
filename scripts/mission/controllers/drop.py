#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 03:10:24 2018

@author: vijayaganesh
"""

from controller import Controller
from geometry_msgs.msg import  PoseStamped
from utilities import Utils
import rospy


class Drop_Controller(Controller):
  def __init__(self,*args, **kwargs):
    self.NAME = "Drop Controller"
    super(Drop_Controller,self).__init__(*args, **kwargs)
    self.drone.land(0,0)
    rospy.loginfo('Dropping the drone')
    rospy.loginfo("Approaching Landing area")
    
  def enter(self):
    self.drone.set_xy_speed(12)
    rospy.loginfo("Entering Drop Mode")
    self.drone.command_offboard()
    self.drone.arm()
    self.home_lat = rospy.get_param("~home_lat")
    self.home_lon = rospy.get_param("~home_lon")
    self.land_lat = rospy.get_param("~land_lat")
    self.land_lon = rospy.get_param("~land_lon")
    self.land_pose = PoseStamped()
    land_local = Utils.convert_global2local(self.home_lat,self.home_lon,self.land_lat,self.land_lon)
    self.land_pose.pose.position.x = land_local[0]
    self.land_pose.pose.position.y = land_local[1]
    self.land_pose.pose.position.z = 5
    
  def handle_track_message(self,msg):
    pass
  
  
  def run(self):
    if abs(Utils.compute_distance(self.land_pose,self.drone.pose)) > 0.5:
        self.drone.setpoint_pose(self.land_pose)
    else:
      self.drone.land(self.land_lat,self.land_lon)
      rospy.loginfo("Mission Complete")
      exit()