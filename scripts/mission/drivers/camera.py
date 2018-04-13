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


class Camera(object):

    processed_image = None  # type: Image

    def __init__(self):
        self.frame = None
        self.raw_image = None
        self.processed_image = None
        pass

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

