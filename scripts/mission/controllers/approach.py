"""
Created on Tue Apr 17 03:18:25 2018

@author: vijayaganesh
"""

from controller import Controller

# Maximum Search speed, in m/s
DEFAULT_MAX_SPEED_XY = 1

class Approach_Controller(Controller):
  def __init__(self,*args,**kwargs):
    super(Approach_Controller,self).__init__(*args,**kwargs)
    self.NAME = "Approach Controller"
    exit()