import os
import re
import subprocess
import time

import kivy
import rospy
from hb_control_msgs.msg import StationKeepingModes
from std_msgs.msg import String

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import StringProperty

import seawolf
from seawolf.utils.customWidgets import CustomTextIndicator


class StateIndicator(CustomTextIndicator):
    current_state = StringProperty("No State")
    depth_lock = False

    def __init__(self, **kwargs):
        super(StateIndicator, self).__init__(**kwargs)
        self.set_text()
        self._state_sub = rospy.Subscriber("~state", String, callback=self._state_cb)
        self._control_mode_sub = rospy.Subscriber(
            "~station_keeping_control_modes", StationKeepingModes, callback=self._control_mode_cb
        )

    def _state_cb(self, msg):
        """Store state and set text accordingly.

        Args:
            msg (String): msg
        """
        self.current_state = msg.data
        self.set_text()

    def _control_mode_cb(self, msg):
        if msg.z_mode == StationKeepingModes.LOCK:
            self.depth_lock = True
        else:
            self.depth_lock = False

        self.set_text()

    def set_text(self):
        text = self.current_state
        # Only append "depth lock" in applicable modes
        if self.depth_lock and any(
            [
                self.current_state.lower().find(mode) >= 0
                for mode in ["manual cleaning", "depth hold", "pose hold", "trajectory following"]
            ]
        ):
            text += " + Depth Lock"

        super(StateIndicator, self).set_text(text)
