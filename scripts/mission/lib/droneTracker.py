#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 13:26:06 2018

@author: vijayaganesh
"""
from position import Positions
from camera import SimCam , RealCam
import cv2
import numpy as np
from wolfdrone.msg import TrackStamped

class DroneTracker(Positions):

  """
    TODO: Kalman Filter based image tracker to adjust the drone velocity on descend to land on top of the drone to be picked up.


  """

  def __init__(self,use_sim = True):
    if use_sim:
      self.cam = SimCam()
    else:
      self.cam = RealCam()
    #tracker_publisher = rospy.Publisher('/wolfdrone/drone_tracker/status',TrackStamped)
  
  def run(self):
    img = self.cam.get_frame()
    temp = img
    if img is not None:
      hsv_img = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
      masked = cv2.inRange(hsv_img,np.array([110,50,50]),np.array([130,255,255]))
      img,contours,hierarchy = cv2.findContours(masked,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        if(w<50 or h<50):
          continue
        cv2.rectangle(temp,(x,y),(x+w,y+h),(0,255,0),5)
      cv2.imshow('hsv_image',temp)
      if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        exit()
