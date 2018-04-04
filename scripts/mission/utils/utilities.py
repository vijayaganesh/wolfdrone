#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 13:53:06 2018

@author: vijayaganesh
"""

import numpy as np
from math import sin,cos,radians,atan2,sqrt

class Utils:
  
  
  """
    Utility function to generate setpoints of the elliptical search area
    l = Length of the Major Axis
    w = length of the minor Axis
    x0,y0 = Center of the Search Area
    step = Distance between two search grid in meters.
    search_altitude = Search altitude in meters
  """
  @staticmethod
  def elliptical_wp(l,w,x0,y0,step = 1,search_altitude = 5):
    wp_list = list()
    n_pts = 2 * l / step
    steps = np.linspace(-l+step,l-step,n_pts)
    wp_list.append((x0-l,y0,search_altitude))
    for k,i in enumerate(steps):
        dy = -k*2
        y = w * (1-(i/(l))**2)**0.5
        wp_list.append((x0+i,y+y0+dy,search_altitude))
        wp_list.append((x0+i,-y+y0+dy,search_altitude))
    wp_list.append((x0+l,y0,search_altitude))   
    wp_list.reverse()
    return wp_list
  
  @staticmethod
  def compute_distance(pose_a,pose_b):
    a_x = pose_a.pose.position.x
    a_y = pose_a.pose.position.y
    a_z = pose_a.pose.position.z
    b_x = pose_b.pose.position.x
    b_y = pose_b.pose.position.y
    b_z = pose_b.pose.position.z  
    return ((a_x - b_x) ** 2 + (a_y - b_y) ** 2 + (a_z - b_z) ** 2) ** 0.5 
    
  @staticmethod
  def compute_global_distance(a_lat,a_long,b_lat,b_long):
      a_lat = radians(a_lat)
      a_long = radians(a_long)
      b_lat = radians(b_lat)
      b_long = radians(b_long) 
      R = 6373.0
      dlat = b_lat - a_lat
      dlong = b_long - a_long
      a = sin(dlat / 2)**2 + cos(a_lat) * cos(b_lat) * sin(dlong / 2)**2
      c = 2 * atan2(sqrt(a), sqrt(1 - a))
      return R*c*1000
  
  @staticmethod
  def convert_global2local(self,a_lat,a_lon,b_lat,b_lon):
    x = Utils.compute_global_distance(a_lat,b_lon,a_lat,a_lon)
    y = Utils.compute_global_distance(b_lat,a_lon,a_lat,a_lon)
    if(b_lat < a_lat):        
        y = -y
    if(b_lon < a_lon):
        x = -x
    return x,y
    
    