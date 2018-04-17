#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 03:21:02 2018

@author: vijayaganesh
"""

import rospy,sys
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/lib')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/drivers')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/nodes')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/controllers')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/utils')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission')

from search import Search_Controller
from vehicle import Vehicle
from wolfdrone.msg import TrackStamped

DEFAULT_CONTROL_LOOP_RATE = 10

class Rescue_Mission():
  def __init__(self,drone):
    rospy.init_node("rescue_mission",log_level=rospy.INFO)
    control_loop_rate = rospy.get_param("~control_loop_rate", DEFAULT_CONTROL_LOOP_RATE)
    rospy.Subscriber("/tracker/track", TrackStamped, self.handle_track_message)
    self.control_loop_rate = rospy.Rate(control_loop_rate)
    self.drone = drone
    self.curr_controller = Search_Controller(self,self.drone)
    self.curr_controller.enter()

  def switch_state(self,new_controller):
    print("Here")
    rospy.loginfo("CONTROLLER TRANSITION: %s --> %s", self.curr_controller.NAME, new_controller.NAME)
    if self.curr_controller is not None:
      self.curr_controller.exit()
    self.curr_controller = new_controller(self,self.drone)
    self.curr_controller.enter()
    
  def handle_track_message(self, msg):
    self.curr_controller.handle_track_message(msg)

  def run(self):
    """
    Spin the ROS event loop, running the controller on each iteration.
    """
    while not rospy.is_shutdown():
        self.curr_controller.run()
        self.control_loop_rate.sleep()


  def cleanup(self):
    pass


if __name__== '__main__':
  drone = Vehicle()
  mission = Rescue_Mission(drone)
  mission.run()