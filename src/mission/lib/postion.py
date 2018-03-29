#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 24 23:42:56 2018

@author: vijayaganesh
"""

import rospy

from geometry_msgs.msg import PoseStamped,TwistStamped

from mavros.srv import StreamRate, StreamRateRequest

class Positions:
  """
    TODO: ADD DOCS
  """
  def __init__(self):
    
    """
    TODO: update self.position and self.orientation to home pose
    """
    self.position = None
    self.orientation = None
    
    
    # Setting the Stream rate fast enough to make control actions.
    rospy.wait_for_service("mavros/set_stream_rate")
    set_stream_rate=rospy.ServiceProxy("mavros/set_stream_rate",StreamRate)
    set_stream_rate(StreamRateRequest.STREAM_POSITION, 50, True)
    set_stream_rate(StreamRateRequest.STREAM_EXTRA1, 50, True)
    
    # Subscriber to process the position
    rospy.Subscriber("/mavros/local_position/local",PoseStamped,self.process_position)
    
    # Subscriber to process the velocity
    rospy.Subscriber("/mavros/local_position/velocity",TwistStamped,self.process_velocity)

  def process_position(self,pose):
    self.position = pose.pose.position
    self.orientation = pose.pose.orientation
  
  
  def process_velocity(self,velocity):
    self.velocity = velocity.vector
    
    
    