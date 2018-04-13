#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 13:26:06 2018

@author: vijayaganesh
"""
from position import Positions
from camera import SimCam , RealCam
import cv2

class DroneTracker(Positions):

  """
    TODO: Kalman Filter based image tracker to adjust the drone velocity on descend to land on top of the drone to be picked up.


  """

  def __init__(self,use_sim = True):
    if use_sim:
      self.cam = SimCam()
    else:
      self.cam = RealCam()
  
  def run(self):
    img = self.cam.get_frame()  
    if img is not None:
      cv2.imshow('Image Feed',self.cam.get_frame())
      if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        exit()
