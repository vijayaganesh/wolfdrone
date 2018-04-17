#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 14:55:35 2018

@author: vijayaganesh
"""

import rospy,sys
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/lib')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/drivers')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/nodes')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/controller')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/utils')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission')

import os
from vehicle import Vehicle
from utilities import Utils
from geometry_msgs.msg import PoseStamped

class Simple_Mission():
  def __init__(self):
    rospy.init_node("simple_mission",log_level=rospy.INFO)
    cwd = os.getcwd()
    print(cwd)
    self.drone = Vehicle()
    self.drone.home_lat = rospy.get_param("~home_lat")
    self.drone.home_lon = rospy.get_param("~home_lon")
    self.drone.drop_lat = rospy.get_param("~drop_lat")
    self.drone.drop_lon = rospy.get_param("~drop_lon")
    self.drone.land_lat = rospy.get_param("~land_lat")
    self.drone.land_lon = rospy.get_param("~land_lon")

  def startup(self):
    l = rospy.get_param("~major_axis")
    w = rospy.get_param("~minor_axis")
    x0 = rospy.get_param("~x_center")
    y0 = rospy.get_param("~y_center")
    alt = rospy.get_param("~alt")
    grid_step = rospy.get_param("~scan_step")

    self.wp_list = Utils.elliptical_wp(l,w,x0,y0,grid_step,alt)
    print(self.wp_list)


  def mission(self):

    ### PX4 rejects the offboard control mode without few setpoint commands already.
    ### So adding arbitrary number of waypoints before making offboard
    rate = rospy.Rate(10)
    print(self.wp_list)
    t = self.wp_list.pop()
    pose = PoseStamped()
    pose.pose.position.x = t[0]
    pose.pose.position.y = t[1]
    pose.pose.position.z = t[2]
    print(pose)
    for i in range(10):
      self.drone.setpoint_pose(pose)
      rate.sleep()
    print("Sent Arbitrary Number of waypoints")
    self.drone.command_mode('OFFBOARD')
    self.drone.arm()


    ### Waypoint Navigation code

    while not len(self.wp_list) == 0:
      print(Utils.compute_distance(pose,self.drone.pose))
      if abs(Utils.compute_distance(pose,self.drone.pose)) > 0.5:
          self.drone.setpoint_pose(pose)
          rate.sleep()
      else:
          t = self.wp_list.pop()
          pose.pose.position.x = t[0]
          pose.pose.position.y = t[1]
          pose.pose.position.z = t[2]

    ### On Scanning the area, proceeding to the drop zone
    drop_pose = PoseStamped()
    drop_local = Utils.convert_global2local(self.drone.home_lat,self.drone.home_lon,self.drone.drop_lat,self.drone.drop_lon)
    drop_pose.pose.position.x = drop_local[0]
    drop_pose.pose.position.y = drop_local[1]
    drop_pose.pose.position.z = 5
    while abs(Utils.compute_distance(drop_pose,self.drone.pose)) > 0.5:
        self.drone.setpoint_pose(drop_pose)
        rate.sleep()
    self.drone.land(self.drone.drop_lat,self.drone.drop_lon)

    #### Once Dropped, Continue to the land location.
    land_pose = PoseStamped()
    land_local = Utils.convert_global2local(self.drone.home_lat,self.drone.home_lon,self.drone.land_lat,self.drone.land_lon)
    land_pose.pose.position.x = land_local[0]
    land_pose.pose.position.y = land_local[1]
    land_pose.pose.position.z = 5
    for i in range(10):
      self.drone.setpoint_pose(land_pose)
      rate.sleep()
    print("Sent Arbitrary Number of waypoints")
    self.drone.command_mode('OFFBOARD')
    while abs(Utils.compute_distance(land_pose,self.drone.pose)) > 0.5:
        self.drone.setpoint_pose(land_pose)
        rate.sleep()
    self.drone.land(self.drone.land_lat,self.drone.land_lon)
    print("Mission Complete")



  def cleanup(self):
    pass


  def run(self):
    # type: () -> object

    self.startup()
    self.mission()
    self.cleanup()


if __name__== '__main__':
  mission = Simple_Mission()
  mission.run()
