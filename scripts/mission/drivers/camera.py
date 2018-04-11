#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 13:34:14 2018

@author: vijayaganesh
"""
import rospy
from positions import Positions

class Camera(object,Positions):
  """
    TODO: 
        1. Image preprocessing
        2. Homogeneous Transformation
        3. Perspective Transformation
        4. Run SSD code to detect the drone
  """
  def __init__(self,ros_topic = None):
    Positions.__init__(self)
    if ros_topic is None:
      ##### TODO Write code to initialize CV Camera
    else:
      rospy.Subscriber('/iris/camera1/camera_raw', Image,self.processImage)
        
  def release:
    pass
  
  def back_project(self, u, v):
      """
      Project a point from (u, v) pixel coordinates to (x, y, z) world coordinates,
      assuming the point lies on the ground plane.
      """
      # Express image point as a column vector in homogeneous coordinates
      P_i = np.matrix((u, v, 1)).T

      # Project the point from image coordinates to a ray in camera coordinates
      K = self.P[:, :3]
      P_c = K.I * P_i

      # Recover 3D point from ray, using the vehicle's altitude to estimate
      # the point's depth in the scene
      d = self.pose.pose.z
      P_c = P_c / P_c[2] * d

      # Build translation vector
      p = self.pose.pose
      c = numpy.matrix((p.x, p.y, p.z)).T

      # Get euler angles from vehicle pose
      o = self.velocity.
      q = [o.x, o.y, o.z, o.w]
      roll, pitch, yaw = euler_from_quaternion(q)

      # Compute rotation matrix, applying rigid body rotations around body axes,
      # in yaw-pitch-roll order, to compute the camera's pose in world coordinates
      # NB: mavros is still working on sign conventions; in the meantime, we need
      #     to negate roll to match mavlink/mavproxy
      R = numpy.matrix(euler_matrix(yaw, pitch, -roll, axes="rzxy"))[:3,:3]

      # Convert from left-handed to right-handed coordinate system
      # (assuming that the camera points down, such that positive z is down)
      L = numpy.diag([1, 1, -1])

      # Rotate and translate the point from left-handed camera coordinates
      # to right-handed world coordinates
      P_w = (R * L * P_c) + c

      return P_w