#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 24 23:38:23 2018

@author: vijayaganesh
"""

from abc import ABC, abstractmethod
class Mission(ABC):
  
  @abstractmethod
  def startup(self):
    pass
  
  @abstractmethod  
  def mission(self):
    pass
  
  @abstractmethod
  def run(self):
    pass
  
  @abstractmethod
  def cleanup(self):
    pass
  