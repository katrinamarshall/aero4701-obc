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
from kivy.properties import ColorProperty, ListProperty, NumericProperty
from kivy.uix.boxlayout import BoxLayout

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("speedIndicator.kv"))


class SpeedIndicator(BoxLayout):
    speed_number = NumericProperty(0)
    speed_line = NumericProperty(0)
    white = ColorProperty([1, 1, 1, 1])

    def __init__(self, **kwargs):
        super(SpeedIndicator, self).__init__(**kwargs)
        self._MAX_SPEED = 0.3  # m/s
        self._timeout = 1  # s
        self._last_time_stamp = 0
        self._odom_sub = rospy.Subscriber("~odometry", Odometry, callback=self._odom_cb)
        self._odom_msg = Odometry()

        update_rate = 10  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))

    def _odom_cb(self, msg):
        self._last_time_stamp = rospy.get_time()
        self._odom_msg = msg

    def _update_graphics_cb(self, *args):
        """Update text and line value based on speed."""

        if self._is_timeout():
            self.speed_number = 0
            self.speed_line = 0
        else:
            speed = round(self._odom_msg.twist.twist.linear.x, 2)
            self.speed_number = abs(speed)

            # Clip at max speed
            speed = min(speed, self._MAX_SPEED)

            # 0.4 is the maximum percentage height of the speed line to work with the kivvy layout
            self.speed_line = (speed / self._MAX_SPEED) * 0.4

    def _is_timeout(self):
        """Check timeout between current time and msg recieved time.

        Returns:
            Bool: timeout flag
        """
        now = rospy.get_time()
        return (now - self._last_time_stamp) > self._timeout
