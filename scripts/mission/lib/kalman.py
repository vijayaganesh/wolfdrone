#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 19 01:36:42 2018

@author: vijayaganesh


This code is inspired from Claymotion's lander project
"""

import numpy as np

class KalmanFilter:
  def __init__(self, F, B, H, x, P, Q, R):
    self.F = F  # state transition matrix
    self.B = B  # control model matrix
    self.H = H  # measurement model matrix
    self.x = x  # state vector
    self.P = P  # state covariance matrix
    self.Q = Q  # process error covariance matrix
    self.R = R  # measurement error covariance matrix
    self.I = np.eye(self.P.shape[0])

  def predict(self, u=0, dt=1):
    """
    Time update: predict future state with control u.
    """
    # Update model and error matrices for current dt
    F = result(self.F, dt)
    B = result(self.B, dt)
    Q = result(self.Q, dt)

    # Compute state prediction
    self.x = F * self.x + B * u

    # Incorporate process error
    self.P = (F * self.P) * F.T + Q

  def update(self, z):
    """
    Measurement update: incorporate measurement z.
    """
    # Compute estimate error (residual)
    y = z - self.H * self.x

    # Project into measurement space
    S = self.H * self.P * self.H.T + self.R

    # Compute Kalman gain
    K = self.P * self.H.T * S.I

    # Compute maximum likelihood state estimate
    self.x = self.x + K * y

    # Incorporate measurement error
    I_KH = self.I - K * self.H
    self.P = (I_KH * self.P) * I_KH.T + (K * self.R) * K.T

def result(f, *args, **kwargs):
  if callable(f):
      return f(*args, **kwargs)
  return f