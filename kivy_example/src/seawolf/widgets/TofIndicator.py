import os
import subprocess
import time

import kivy
import rospy
from hb_drivers_msgs.msg import TimeOfFlightArray
from kivy.clock import Clock

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ColorProperty, ListProperty, NumericProperty
from kivy.uix.relativelayout import RelativeLayout

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("tofIndicator.kv"))


class TofIndicator(RelativeLayout):
    bot_graphic_file = "top-view.png"
    tof_a_color = ListProperty([1, 1, 1, 1])
    tof_b_color = ListProperty([1, 1, 1, 1])
    tof_c_color = ListProperty([1, 1, 1, 1])
    tof_d_color = ListProperty([1, 1, 1, 1])
    tof_e_color = ListProperty([1, 1, 1, 1])
    tof_f_color = ListProperty([1, 1, 1, 1])
    tof_g_color = ListProperty([1, 1, 1, 1])
    tof_h_color = ListProperty([1, 1, 1, 1])
    tof_a_val = NumericProperty(0)
    tof_b_val = NumericProperty(0)
    tof_c_val = NumericProperty(0)
    tof_d_val = NumericProperty(0)
    tof_e_val = NumericProperty(0)
    tof_f_val = NumericProperty(0)
    tof_g_val = NumericProperty(0)
    tof_h_val = NumericProperty(0)
    white = ColorProperty([1, 1, 1, 1])
    green = ColorProperty([0, 0.6, 0, 1])
    red = ColorProperty([0.8, 0, 0, 1])
    teal = ColorProperty([0, 0.95, 0.85, 1])

    def __init__(self, **kwargs):
        super(TofIndicator, self).__init__(**kwargs)

        self._GREEN_THRESHOLD = 0.09  # m
        self._RED_THRESHOLD = 0.15  # m

        self._timeout = 1  # s
        self._tofs_sub = rospy.Subscriber("~tofs", TimeOfFlightArray, callback=self._tof_cb)
        self._tofs_msg = TimeOfFlightArray()

        self._last_tof_times = {
            "tof_a_link": 0,
            "tof_b_link": 0,
            "tof_c_link": 0,
            "tof_d_link": 0,
            "tof_e_link": 0,
            "tof_f_link": 0,
            "tof_g_link": 0,
            "tof_h_link": 0,
        }

        update_rate = 10  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))

    def _tof_cb(self, msg):
        """Store tofs msg and recieved msg time.

        Args:
            msg (TimeOfFlightArray): msg
        """
        for tof in msg.ranges:
            self._last_tof_times[tof.header.frame_id] = rospy.get_time()
        self._tofs_msg = msg

    def _update_graphics_cb(self, *args):
        """
        Updates TOF displays for recieved TOF msgs.
        """
        for tof_id, time in self._last_tof_times.items():

            if self._is_timeout(time):
                self._set_color_and_line_length(tof_id, 0, self.white)
            else:
                tof_msg = self._get_single_tof_msg_from_id(tof_id)
                if tof_msg:
                    self._update_tof_display(tof_msg)

    def _update_tof_display(self, msg):
        """Updates the TOF color and line length based on set thresholds and normalised range."""
        tof_id = self._extract_tof_id(msg.header.frame_id)
        tof_range = msg.range
        tof_norm_range = self._normalise_range(tof_range)

        # Detemine color indicator from threshold
        if tof_range < self._GREEN_THRESHOLD:
            color = self.green
        elif tof_range < self._RED_THRESHOLD:
            color = self.red
        elif tof_range is float("inf"):
            color = self.white
        else:
            color = self.white

        self._set_color_and_line_length(tof_id, tof_norm_range, color)

    def _set_color_and_line_length(self, tof_id, tof_norm_range, color):
        """
        Sets the TOF display color and line length.
        """
        if hasattr(self, "tof_" + tof_id.lower() + "_val"):
            setattr(self, "tof_" + tof_id.lower() + "_val", tof_norm_range)
        if hasattr(self, "tof_" + tof_id.lower() + "_color"):
            setattr(self, "tof_" + tof_id.lower() + "_color", color)

    def _normalise_range(self, tof_range):
        """
        Normalise the range between 0-1 based on maximum tof range.
        """
        norm_range = tof_range / self._RED_THRESHOLD
        if norm_range > 1:
            norm_range = 1
        return norm_range

    def _extract_tof_id(self, frame_id):
        """
        Extracts tof_id: x from string format: "tof_x_link".
        """
        split_strings = frame_id.split("_")
        tof_id = split_strings[1]
        return tof_id

    def _get_single_tof_msg_from_id(self, tof_id):
        """Get single tof msg from tof array msg based on id.

        Args:
            tof_id (string): tof_id

        Returns:
            TimeOfFlight: tof msg
        """
        for tof in self._tofs_msg.ranges:
            if tof.header.frame_id == tof_id:
                return tof
        return None

    def _is_timeout(self, time):
        """Check timeout between current time and msg recieved time.

        Returns:
            Bool: timeout flag
        """
        now = rospy.get_time()
        return (now - time) > self._timeout
