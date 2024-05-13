import os
import subprocess
import time

import kivy
import rospy
from kivy.clock import Clock
from nav_msgs.msg import Odometry

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import NumericProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from WarningPopups import DepthWarningPopup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("depthIndicator.kv"))

# Known bug where barometer reading 0 pressure, thus converting to 10m+ depth.
WARNING_DEPTH = 10


class DepthIndicator(BoxLayout):
    depth = NumericProperty(0)
    min_depth = NumericProperty(0)
    max_depth = NumericProperty(-5)

    def __init__(self, **kwargs):
        super(DepthIndicator, self).__init__(**kwargs)

        self._depth_increment = -5
        self._depth_warning_popup = DepthWarningPopup(WARNING_DEPTH)

        self._timeout = 1
        self._last_time_stamp = 0
        self._odom_sub = rospy.Subscriber("~odometry", Odometry, callback=self._odom_cb)
        self._odom_msg = Odometry()

        update_rate = 10  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))
        self.saved_depth = 0

    def _odom_cb(self, msg):
        """Store msg and recieved msg time.

        Args:
            msg (Odometry): msg
        """
        self._last_time_stamp = rospy.get_time()
        self._odom_msg = msg

    def _update_graphics_cb(self, *args):
        """Update text, slider value and scaling based on depth."""
        if self._is_timeout():
            depth = 0
            self._set_depth(depth)
        else:
            depth = round(self._odom_msg.pose.pose.position.z, 2)
            self._set_depth(depth)

    def _set_depth(self, depth):
        """Set depth by setting slider value and scale.

        Args:
            depth (float): depth
        """
        self._rescale_slider(depth)
        self.depth = depth

    def _rescale_slider(self, depth):
        """Rescale sliler based on depth.

        Args:
            depth (float): depth
        """
        # Rescale slider to be between 0 and max_depth (5m, 10m, 15m etc) when underwater.
        if depth < 0 and depth < self.max_depth:
            self.min_depth = 0
            self.max_depth += self._depth_increment
        elif depth < 0 and depth > (self.max_depth - self._depth_increment):
            self.min_depth = 0
            self.max_depth -= self._depth_increment
        # Rescale slider to be between 5 and 0 when above water.
        elif depth > 0:
            self.min_depth = 5
            self.max_depth = 0

        # Show depth warning to indicate potential bug has occured
        if depth > WARNING_DEPTH:
            self._depth_warning_popup.show()

    def _is_timeout(self):
        """Check timeout between current time and msg recieved time.

        Returns:
            Bool: timeout flag
        """
        now = rospy.get_time()
        return (now - self._last_time_stamp) > self._timeout
