#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 13:26:06 2018

@author: vijayaganesh
"""
from position import Positions
from camera import SimCam , RealCam
import cv2,rospy
import numpy as np
from wolfdrone.msg import TrackStamped
import time
from kalman import KalmanFilter

class DroneTracker(Positions):

  """
    TODO: Kalman Filter based image tracker to adjust the drone velocity on descend to land on top of the drone to be picked up.
    
  """

  def __init__(self,use_sim = True):
    if use_sim:
      self.cam = SimCam()
    else:
      self.cam = RealCam()
    self.tracker_publisher = rospy.Publisher('/wolfdrone/drone_tracker',TrackStamped,queue_size = 10)
    self.tracking = False
    self.last_frame_time = 0
    self.last_seen_time = 0
    self.initialize_kalman()
  
  def run(self):
    now = time.time()
    
    dt = now - self.last_frame_time if self.last_frame_time else 0
    self.last_frame_time = now    
    
    frame = self.cam.get_frame()
    if frame is not None:
      bp_points = self.detect_target(frame)
      see_now = bp_points is not None
      see_previous = now - self.last_seen_time < 2
      
      if see_now :
        self.last_seen_time = now
        
      if see_now and not see_previous and not self.tracking:
        x, y, z = bp_points
        self.initialize_kalman(x=x, y=y, z=z)
      
     # Time update
      if self.tracking or (see_now and see_previous):
        self.tracker.predict(dt=dt)

      # Measurement update
      if see_now and (see_previous or self.tracking):
        self.tracker.update(bp_points)
          
      position = self.tracker.x[[0, 2, 4], 0].T.tolist()[0]
      velocity = self.tracker.x[[1, 3, 5], 0].T.tolist()[0]
      
      # Determine whether we are certain about the target position
      certain = (self.tracker.P < 2).all()

      if certain and not self.tracking:
        rospy.loginfo("Found target at (%6.4f, %6.4f, %6.4f)", *position)
      elif not certain and self.tracking:
        rospy.loginfo("Lost target")

      self.tracking = certain

      self.publish_track(position, velocity)
      
    
  def publish_track(self, position, velocity):
    """
    Publish a TrackStamped message containing relative position
    (and eventually velocity) of the tracked object.
    """
    msg = TrackStamped()
    msg.track.tracking.data = self.tracking
  
    if self.tracking:
        msg.track.position.x = position[0]
        msg.track.position.y = position[1]
        msg.track.position.z = position[2]
        msg.track.velocity.x = velocity[0]
        msg.track.velocity.y = velocity[1]
        msg.track.velocity.z = velocity[2]
  
    self.tracker_publisher.publish(msg)
  
  def initialize_kalman(self, x=0, vx=0, y=0, vy=0, z=0, vz=0, p=10, q=1, r=1):
    F = lambda dt: np.matrix([
            [  1, dt,  0,  0,  0,  0  ],  # x' = x + vx*dt
            [  0,  1,  0,  0,  0,  0  ],  # vx' = vx
            [  0,  0,  1, dt,  0,  0  ],  # y' = y + vy*dt
            [  0,  0,  0,  1,  0,  0  ],  # vy' = vy
            [  0,  0,  0,  0,  1, dt  ],  # z' = z + vz*dt
            [  0,  0,  0,  0,  0,  1  ],  # vz' = vz
    ])

    # Control model 
    B = 0

    # Measurement model
    H = np.matrix([
        [  1, 0, 0, 0, 0, 0  ], # x
        [  0, 0, 1, 0, 0, 0  ], # y
        [  0, 0, 0, 0, 1, 0  ], # z
    ])

    # Initial state vector
    x0 = np.matrix([ x, vx, y, vy, z, vz ]).T

    # Initial state covariance
    P0 = np.matrix(np.eye(6)) * p

    # Process error covariance (continuous white noise acceleration model)
    Q = lambda dt: np.matrix([
        [  dt**3/3.0,  dt**2/2.0,          0,          0,          0,          0  ],
        [  dt**2/2.0,         dt,          0,          0,          0,          0  ],
        [          0,          0,  dt**3/3.0,  dt**2/2.0,          0,          0  ],
        [          0,          0,  dt**2/2.0,         dt,          0,          0  ],
        [          0,          0,          0,          0,  dt**3/3.0,  dt**2/2.0  ],
        [          0,          0,          0,          0,  dt**2/2.0,         dt  ],
    ]) * q

    # Measurement error covariance
    R = np.matrix(np.eye(3)) * r

    self.tracker = KalmanFilter(F, B, H, x0, P0, Q, R)
  
  
  def detect_target(self,frame):
    if cv2.waitKey(1) & 0xFF == ord('q'):
      cv2.destroyAllWindows()
    hsv_img = cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
    masked = cv2.inRange(hsv_img,np.array([110,50,50]),np.array([130,255,255]))
    img,contours,hierarchy = cv2.findContours(masked,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours is None or len(contours) == 0 : return
      
    for c in contours:
      x,y,w,h = cv2.boundingRect(c)
      if(w<25 or h<25):
        continue
      u,v = (2 * x + w)/2 , (2 * y + h)/2
      cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),5)
      cv2.imshow('frame',frame)
      return self.cam.back_project(u,v)
    
    
  
  
  
