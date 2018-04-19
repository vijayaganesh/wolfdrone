#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 13:34:14 2018

@author: vijayaganesh
"""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from position import Positions
from tf.transformations import euler_from_quaternion, euler_matrix

DEFAULT_CAMERA_MATRIX = [[476.70143780997665, 0.0, 400.5, 0],[0.0, 476.70143780997665, 400.5, 0],[0, 0, 1, 0]]



class Camera(object,Positions):

    processed_image = None  # type: Image

    def __init__(self):
        Positions.__init__(self)
        self.frame = None
        self.raw_image = None
        self.processed_image = None
        self.P = np.matrix(rospy.get_param("~camera_matrix",DEFAULT_CAMERA_MATRIX))
        self.P[1,1] = -self.P[1,1]
        
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
        d = self.pose.pose.position.z
        P_c = P_c / P_c[2] * d
        p = self.pose.pose.position
        c = np.matrix((p.x, p.y, p.z)).T
        o = self.pose.pose.orientation
        q = [o.x, o.y, o.z, o.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        # Compute rotation matrix, applying rigid body rotations around body axes,
        # in yaw-pitch-roll order, to compute the camera's pose in world coordinates
        R = np.matrix(euler_matrix(yaw, pitch, -roll, axes="rzxy"))[:3,:3]
        # Convert from left-handed to right-handed coordinate system
        # (assuming that the camera points down, such that positive z is down)
        L = np.diag([1, 1, -1])

        # Rotate and translate the point from left-handed camera coordinates
        # to right-handed world coordinates
        P_w = (R * L * P_c) + c
        return P_w    
    
    
    def pre_process(self):
        self.processed_image = self.raw_image

    def release(self):
        pass

    def get_frame(self):
        return self.processed_image


class SimCam(Camera):
    def __init__(self):
        super(SimCam, self).__init__()
        rospy.Subscriber('/iris/camera1/camera_raw', Image, self.image_callback)

    def image_callback(self, image):
        np_arr = np.fromstring(image.data, np.uint8)
        image = np.reshape(np_arr,(800,800,3))
        self.raw_image = image
        self.pre_process()

    def get_frame(self):
        return self.processed_image


class RealCam(Camera):
    def __init__(self):
        super(RealCam,self).__init__()
        self.cap = cv2.VideoCapture(0)

    def get_frame(self):
        self.raw_image = self.cap.read()
        self.pre_process()
        assert isinstance(self.processed_image, Image)
        return self.processed_image

