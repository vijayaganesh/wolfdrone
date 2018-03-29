#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 14:55:35 2018

@author: vijayaganesh
"""

import rospy
from mission import Mission
from mission.lib.vehicle import Vehicle
from mission.utils.utilities.Utils import elliptical_wp,compute_distance, convert_global2local
from geometry_msgs.msg import PoseStamped

class Simple_Mission(Mission):
  def __init__(self):
    rospy.init_node("Simple_Mission")
    self.drone = Vehicle()
    self.drone.home_lat = rospy.get_param("home_lat")
    self.drone.home_lon = rospy.get_param("home_lon")
    self.drone.drop_lat = rospy.get_param("drop_lat")
    self.drone.drop_lon = rospy.get_param("drop_lon")
    self.drone.land_lat = rospy.get_param("land_lat")
    self.drone.land_lon = rospy.get_param("land_lon")
    
  def startup(self):
    l = rospy.get_param("major_axis")
    w = rospy.get_param("minior_axis")
    x0 = rospy.get_param("x_center")
    y0 = rospy.get_param("y_center")
    alt = rospy.get_param("alt")
    grid_step = rospy.get_param("scan_step")
    
    self.wp_list = elliptical_wp(l,w,x0,y0,grid_step,alt)
    
  
  def mission(self):
    
    ### PX4 rejects the offboard control mode without few setpoint commands already.
    ### So adding arbitrary number of waypoints before making offboard
    rate = rospy.Rate(10)
    t = self.wp_list.pop()
    pose = PoseStamped()
    pose.pose.position.x = t[0]
    pose.pose.position.y = t[1]
    pose.pose.position.z = t[2]
    for i in range(5):
      self.drone.setpoint_pose(pose)
      rate.sleep()
    self.drone.command_mode('OFFBOARD')
    self.drone.arm(True)
    
    
    ### Waypoint Navigation code    
    
    while not len(self.wp_list) == 0:
      if abs(compute_distance(pose,self.drone.pose)) > 0.5:
          self.drone.setpoiint_pose(pose)
          rate.sleep()
      else:
          t = self.wp_list.pop()
          pose.pose.position.x = t[0]
          pose.pose.position.y = t[1]
          pose.pose.position.z = t[2]
    
    ### On Scanning the area, proceeding to the drop zone
    drop_pose = PoseStamped()
    drop_local = convert_global2local(self.drone.drop_lat,self.drone.drop_lon)
    drop_pose.pose.position.x = drop_local[0]
    drop_pose.pose.position.y = drop_local[1]
    drop_pose.pose.position.z = 40
    while abs(compute_distance(drop_pose,self.drone.pose)) > 0.5:
        self.drone.setpoiint_pose(drop_pose)
        rate.sleep()
    self.drone.land(self.drone.drop_lat,self.drone.drop_lon)
    
    #### Once Dropped, Continue to the land location.
    self.drone.command_mode('OFFBOARD')
    land_pose = PoseStamped()
    land_local = convert_global2local(self.drone.land_lat,self.drone.land_lon)
    land_pose.pose.position.x = land_local[0]
    land_pose.pose.position.y = land_local[1]
    land_pose.pose.position.z = 40
    while abs(compute_distance(land_pose,self.drone.pose)) > 0.5:
        self.drone.setpoiint_pose(land_pose)
        rate.sleep()
    self.drone.land(self.drone.land_lat,self.drone.land_lon)
    print("Mission Complete")
    
  
  
  def cleanup(self):
    pass
  
  
  def run(self):
    
    self.startup()
    self.mission()
    self.cleanup()
    
     
if __name__== '__main__':
  mission = Simple_Mission()
  mission.run()