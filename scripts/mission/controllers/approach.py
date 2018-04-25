"""
Created on Tue Apr 17 03:18:25 2018

@author: vijayaganesh
"""

from controller import Controller
from descend import Descend_Controller
from geometry_msgs.msg import TwistStamped
import rospy
import time
import numpy as np

# Maximum Search speed, in m/s
DEFAULT_MAX_SPEED_XY = 1.5
DEFAULT_MAX_ACCEL_XY = 0.8
DEFAULT_APPROACH_KP = 0.01
DEFAULT_APPROACH_KI = 0
DEFAULT_APPROACH_KD = 0.01
DEFAULT_APPROACH_PROXIMITY = 30

class Approach_Controller(Controller):
  def __init__(self,*args,**kwargs):
    super(Approach_Controller,self).__init__(*args,**kwargs)
    self.NAME = "Approach Controller"
    self.approach_speed = rospy.get_param("~approach_speed",DEFAULT_MAX_SPEED_XY)
    self.approach_accel = rospy.get_param("~approach_accel",DEFAULT_MAX_ACCEL_XY)
    self.approach_kp = rospy.get_param("~approach_kp",DEFAULT_APPROACH_KP)
    self.approach_ki = rospy.get_param("~approach_ki",DEFAULT_APPROACH_KI)
    self.approach_kd = rospy.get_param("~approach_kd",DEFAULT_APPROACH_KD)
    self.approach_proximity = rospy.get_param("~approach_proximity",DEFAULT_APPROACH_PROXIMITY)
    self.prev_error_x, self.prev_error_y = 0 , 0
    self.cumm_error_x, self.cumm_error_y = 0 , 0
    
#    brake = TwistStamped()
#    rospy.loginfo("Brakes Applied")
#    self.vel_twist = None
#    rate = rospy.Rate(50)
#    for i in range(100):
#      self.drone.setpoint_vel(brake)
#      rate.sleep()  
    self.drone.setpoint_vel(TwistStamped())
    time.sleep(1)
    rospy.loginfo("Commencing Precision Landing")
    self.switch_count = 0
  
  
  
  def enter(self):
    self.vel_twist = None
    self.damping_factor = 0
    
  def handle_track_message(self,msg):

#    drone_pos = self.drone.pose.pose.position
    drone_vel = self.drone.velocity.twist.linear
    
    target_pos = msg.track.position
#    target_vel = msg.track.velocity
    
    
    ## Computing Position Error
    error_x = -target_pos.x + 400
    error_y = -target_pos.y + 400
    
    distance = np.sqrt(error_x**2 + error_y**2)    
    
    kp = self.approach_kp# * self.damping_factor
    diff_x , diff_y = error_x - self.prev_error_x , error_y - self.prev_error_y
    vel_y = kp * error_x + self.approach_ki * self.cumm_error_x +self.approach_kd * diff_x
    vel_x = kp * error_y + self.approach_ki * self.cumm_error_y +self.approach_kd * diff_y
    self.damping_factor = min(self.damping_factor + 0.01, 1.00)
    
    speed = np.sqrt(vel_x**2 + vel_y**2)
    if speed > self.approach_speed:
      vel_x = vel_x * self.approach_speed / speed
      vel_y = vel_y * self.approach_speed / speed
    
    drone_speed = np.sqrt(drone_vel.x**2 + drone_vel.y**2)    
    
    proximity_check   = distance < self.approach_proximity
    stability_check = drone_speed < self.approach_speed
    
#    print((proximity_check,stability_check))
    if msg.track.tracking.data:
      if proximity_check and stability_check:
        self.switch_count = min(100,self.switch_count + 1)
      else:
        self.switch_count = max(0,self.switch_count - 1)
      
    print("stability index: "+repr(self.switch_count))
    if self.switch_count >= 100:  
      self.mission.switch_state(Descend_Controller(self.mission,self.drone))
    
    self.vel_twist = TwistStamped()
    if msg.track.tracking.data:
      self.vel_twist.twist.linear.x = vel_x
      self.vel_twist.twist.linear.y = vel_y
    self.prev_error_x , self.prev_error_y = error_x , error_y
    self.cumm_error_x = max(50 , self.cumm_error_x + error_x)
    self.cumm_error_y = max(50 , self.cumm_error_y + error_x)
    
    
  
  def run(self):
    if self.vel_twist is not None:
      self.drone.setpoint_vel(self.vel_twist)
      
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    