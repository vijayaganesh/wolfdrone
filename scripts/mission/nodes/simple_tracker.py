#!/usr/bin/env python
import rospy, sys

sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/lib')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/drivers')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/nodes')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/controller')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission/utils')
sys.path.append('/home/vijayaganesh/catkin_ws/src/wolfdrone/scripts/mission')

from droneTracker import DroneTracker

class TrackerNode:

    def __init__(self):

        rospy.init_node('Simple_Tracker', anonymous=True)
        tracker = DroneTracker(use_sim = True)
        while not rospy.is_shutdown():
          tracker.run()
        


if __name__ == '__main__':
    TrackerNode()


