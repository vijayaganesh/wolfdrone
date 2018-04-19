"""
Created on Thu Apr 10 14:55:35 2018

@author: vijayaganesh
"""

import rospy
#from geometry_msgs.msg import PoseStamped

from controller import Controller
#from approach import Approach_Controller

class Descend_Controller(Controller):
  def __init__(self, *args, **kwargs):
    self.NAME = "Descend Controller"
    super(Descend_Controller,self).__init__(*args, **kwargs)
    
  def enter():
    rospy.loginfo("Entered Descend Mode")
    

