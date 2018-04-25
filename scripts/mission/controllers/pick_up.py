#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 03:11:00 2018

@author: vijayaganesh
"""

from controller import Controller
from drop import Drop_Controller
from geometry_msgs.msg import  PoseStamped
from utilities import Utils
import rospy


class Pickup_Controller(Controller):
  def __init__(self,*args, **kwargs):
    self.NAME = "Pickup Controller"
    super(Pickup_Controller,self).__init__(*args, **kwargs)
    self.drone.land(0,0)
    rospy.loginfo('Picking up the drone')
    rospy.loginfo("Approaching Drop Zone")
    
  def enter(self):
    self.drone.set_xy_speed(12)
    self.home_lat = rospy.get_param("~home_lat")
    self.home_lon = rospy.get_param("~home_lon")
    self.drop_lat = rospy.get_param("~drop_lat")
    self.drop_lon = rospy.get_param("~drop_lon")
    self.drop_pose = PoseStamped()
    drop_local = Utils.convert_global2local(self.home_lat,self.home_lon,self.drop_lat,self.drop_lon)
    self.drop_pose.pose.position.x = drop_local[0]
    self.drop_pose.pose.position.y = drop_local[1]
    self.drop_pose.pose.position.z = 5
    
  def handle_track_message(self,msg):
    pass
  
  
  def run(self):
    if abs(Utils.compute_distance(self.drop_pose,self.drone.pose)) > 0.5:
        self.drone.setpoint_pose(self.drop_pose)
    else:
      self.mission.switch_state(Drop_Controller(self.mission,self.drone))
      
  