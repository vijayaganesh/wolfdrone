#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 24 23:38:23 2018

@author: vijayaganesh
"""

import abc
class Mission:
  __metaclass__ = abc.ABCMeta
  
  @abc.abstractmethod
  def startup(self):
    pass
  
  @abc.abstractmethod  
  def mission(self):
    pass
  
  @abc.abstractmethod
  def run(self):
    pass
  
  @abc.abstractmethod
  def cleanup(self):
    pass
  
