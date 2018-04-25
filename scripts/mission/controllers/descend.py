"""
Created on Thu Apr 10 14:55:35 2018

@author: vijayaganesh
"""

import rospy
from geometry_msgs.msg import  TwistStamped
from pick_up import Pickup_Controller
import numpy as np
from controller import Controller

DEFAULT_MAX_SPEED_XY = 1.5
DEFAULT_MAX_ACCEL_XY = 0.8
DEFAULT_DESCEND_KP = 0.02
DEFAULT_DESCEND_KI = 0
DEFAULT_DESCEND_KD = 0.02
DEFAULT_DESCEND_PROXIMITY = 30

class Descend_Controller(Controller):
  def __init__(self, *args, **kwargs):
    self.NAME = "Descend Controller"
    super(Descend_Controller,self).__init__(*args, **kwargs)
    self.descend_speed = rospy.get_param("~descend_speed",DEFAULT_MAX_SPEED_XY)
    self.descend_accel = rospy.get_param("~descend_accel",DEFAULT_MAX_ACCEL_XY)
    self.descend_kp = rospy.get_param("~descend_kp",DEFAULT_DESCEND_KP)
    self.descend_ki = rospy.get_param("~descend_ki",DEFAULT_DESCEND_KI)
    self.descend_kd = rospy.get_param("~descend_kd",DEFAULT_DESCEND_KD)
    self.descend_proximity = rospy.get_param("~descend_proximity",DEFAULT_DESCEND_PROXIMITY)
    self.prev_error_x, self.prev_error_y = 0 , 0
    self.cumm_error_x, self.cumm_error_y = 0 , 0
    self.switch_count = 0
    
  def enter(self):
    rospy.loginfo("Entered Descend Mode")
    self.vel_twist = None
    self.damping_factor = 0
    self.curr_altitude = self.drone.pose.pose.position.z
    
    
  def handle_track_message(self,msg):
    
    self.vel_twist = TwistStamped()
    self.curr_altitude = self.drone.pose.pose.position.z    
    drone_vel = self.drone.velocity.twist.linear
    
    target_pos = msg.track.position
#    target_vel = msg.track.velocity
    
    
    ## Computing Position Error
    error_x = -target_pos.x + 400
    error_y = -target_pos.y + 400
    
    distance = np.sqrt(error_x**2 + error_y**2)    
    
    kp = self.descend_kp * self.curr_altitude/10
    kd = self.descend_kd * self.curr_altitude/10
    diff_x , diff_y = error_x - self.prev_error_x , error_y - self.prev_error_y
    vel_y = kp * error_x + self.descend_ki * self.cumm_error_x +kd * diff_x
    vel_x = kp * error_y + self.descend_ki * self.cumm_error_y +kd * diff_y
    self.damping_factor = min(self.damping_factor + 0.01, 1.00)
    
    speed = np.sqrt(vel_x**2 + vel_y**2)
    if speed > self.descend_speed:
      vel_x = vel_x * self.descend_speed / speed
      vel_y = vel_y * self.descend_speed / speed
    
    drone_speed = np.sqrt(drone_vel.x**2 + drone_vel.y**2)    
    
    proximity_check   = distance < self.descend_proximity
    stability_check = drone_speed < self.descend_speed
    
#    print((proximity_check,stability_check))
    if msg.track.tracking.data:
      if proximity_check and stability_check:
        self.switch_count = min(100,self.switch_count + 1)
      else:
        self.switch_count = max(0,self.switch_count - 1)
    else:
      self.vel_twist.twist.linear.z = 0.2
      self.switch_count = 0
      
    print("stability index: "+repr(self.switch_count))
    
    if self.switch_count >= 100:
      self.vel_twist.twist.linear.z = -0.2
    else:
      self.vel_twist.twist.linear.z = 0
      
    if msg.track.tracking.data:
      self.vel_twist.twist.linear.x = vel_x
      self.vel_twist.twist.linear.y = vel_y
    self.prev_error_x , self.prev_error_y = error_x , error_y
    self.cumm_error_x = max(50 , self.cumm_error_x + error_x)
    self.cumm_error_y = max(50 , self.cumm_error_y + error_x)
  
  def run(self):
    if self.curr_altitude > 0.5:
      if self.vel_twist is not None:
        self.drone.setpoint_vel(self.vel_twist)
    else:
      self.mission.switch_state(Pickup_Controller(self.mission,self.drone))

