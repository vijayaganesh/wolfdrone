#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 24 23:42:56 2018

@author: vijayaganesh
"""

import rospy

from geometry_msgs.msg import PoseStamped,TwistStamped,Pose,Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, StreamRateRequest


class Positions:
  """
    TODO: ADD DOCS
  """
  def __init__(self):
    
    """
    TODO: update self.position and self.orientation to home pose
    """
#    self.pose.pose.position = None
#    self.pose.pose.orientation = None
    
    self.pose = PoseStamped()
    self.velocity = TwistStamped()
    # Setting the Stream rate fast enough to make control actions.
    rospy.wait_for_service("mavros/set_stream_rate")
    set_stream_rate=rospy.ServiceProxy("mavros/set_stream_rate",StreamRate)
    set_stream_rate(StreamRateRequest.STREAM_POSITION, 50, True)
    set_stream_rate(StreamRateRequest.STREAM_EXTRA1, 50, True)
    
    # Subscriber to process the position
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.process_position)
    
    # Subscriber to process the state
    rospy.Subscriber("/mavros/state",State,self.process_state)
    
    # Subscriber to process the velocity
    rospy.Subscriber("/mavros/local_position/velocity",TwistStamped,self.process_velocity)
    
  def process_state(self,state):
    self.state = state

  def process_position(self,pose):
    self.pose = pose
  
  
  def process_velocity(self,velocity):
    self.velocity = velocity
    
    
    
