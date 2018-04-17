#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 03:18:25 2018

@author: vijayaganesh
"""

class Controller(object):
    """
    Encapsulates the control logic for each stage of flight
    within its own controller class.
    """
    def __init__(self, commander, vehicle):
        self.commander = commander
        self.drone = vehicle

    def enter(self):
        """
        Called by the commander when transitioning to this state.
        """
        pass

    def exit(self):
        """
        Called by the commander when transitioning from this state.
        """
        pass

    def run(self):
        """
        Called by the commander once per control loop.
        """
        pass

    def handle_track_message(self, msg):
        """
        Called by the commander when it receives a TrackStamped message.
        """
        pass 