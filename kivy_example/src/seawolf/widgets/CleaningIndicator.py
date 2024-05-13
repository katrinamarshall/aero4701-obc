import os
import subprocess
import time

import kivy
import rospy
from hb_common_msgs.msg import CleaningFeedbackArray
from kivy.clock import Clock

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ColorProperty, StringProperty
from kivy.uix.relativelayout import RelativeLayout
from std_msgs.msg import Float32

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("cleaningIndicator.kv"))


class CleaningIndicator(RelativeLayout):
    bot_graphic_file = "bottom-view.png"
    cc_1_color = ColorProperty([0, 0, 0, 0])
    cc_2_color = ColorProperty([0, 0, 0, 0])
    cc_3_color = ColorProperty([0, 0, 0, 0])
    cc_4_color = ColorProperty([0, 0, 0, 0])

    desired_rpm_text = StringProperty("")
    cc_1_text = StringProperty("")
    cc_2_text = StringProperty("")
    cc_3_text = StringProperty("")
    cc_4_text = StringProperty("")
    black = ColorProperty([0, 0, 0, 1])
    green = ColorProperty([0, 0.6, 0, 1])
    red = ColorProperty([0.8, 0, 0, 1])

    def __init__(self, **kwargs):
        # Asset file names
        self.bot_graphic_path = seawolf.form_assets_path(self.bot_graphic_file)

        super(CleaningIndicator, self).__init__(**kwargs)

        self._RPM_THRESHOLD = 10
        self._timeout = 1  # s
        self._last_desired_rpm_time = 0
        self._last_feedback_time = {1: 0, 2: 0, 3: 0, 4: 0}
        self._cleaning_feedback_sub = rospy.Subscriber(
            "~cleaning_feedback", CleaningFeedbackArray, callback=self._cleaning_feedback_cb
        )
        self._cleaning_desired_rpm_sub = rospy.Subscriber(
            "~cleaning_desired_rpm", Float32, callback=self._desired_rpm_cb
        )
        self._cleaning_feedback_msg = CleaningFeedbackArray()
        self._desired_rpm_msg = Float32()

        update_rate = 10  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))

    def _cleaning_feedback_cb(self, msg):
        """Store feedback msg and recieved msg time.

        Args:
            msg (CleaningFeedbackArray): msg
        """
        for feedback_msg in msg.status:
            self._last_feedback_time[feedback_msg.id] = rospy.get_time()
        self._cleaning_feedback_msg = msg

    def _desired_rpm_cb(self, msg):
        """Store feedback msg and recieved msg time.

        Args:
            msg (Float32): msg
        """
        self._last_desired_rpm_time = rospy.get_time()
        self._desired_rpm_msg = msg

    def _update_graphics_cb(self, *args):
        """Updates all cleaning indicators."""

        # Update desired rpm indicator
        if self._is_timeout(self._last_desired_rpm_time):
            setattr(self, "desired_rpm_text", "")
        else:
            self._update_desired_rpm_display(self._desired_rpm_msg)

        # Update cleaning feedback indicators
        for cc_id, time in self._last_feedback_time.items():
            if self._is_timeout(time):
                self._set_cleaning_core_color_and_text(cc_id, self.black, "")
            else:
                try:
                    self._update_cleaning_feedback_display(
                        self._cleaning_feedback_msg.status[cc_id - 1]
                    )
                except IndexError:
                    pass

    def _update_desired_rpm_display(self, msg):
        """Update desired rpm display.

        Args:
            msg (Float32): msg
        """
        desired_rpm = int(msg.data)
        if desired_rpm > 0:
            text = "+{}".format(desired_rpm)
        elif desired_rpm < 0:
            text = "{}".format(desired_rpm)
        else:
            text = ""

        setattr(self, "desired_rpm_text", text)

    def _update_cleaning_feedback_display(self, msg):
        """Update a cleaning core feedback display based on recieved feedback.

        Args:
            msg (CleaningFeedbackArray): msg
        """
        cc_id = msg.id
        rpm = msg.feedback.rpm
        errors = msg.feedback.errors

        # Set cleaning core green if above rpm threshold
        if self._above_rpm_threshold(rpm):
            self._set_cleaning_core_color_and_text(cc_id, self.green, str(int(abs(rpm))))
        else:
            self._set_cleaning_core_color_and_text(cc_id, self.black, "")

        # Set cleaning core red if error code is detected
        detected_errors = [e for e in errors if e != 0]
        if detected_errors:
            self._set_cleaning_core_color_and_text(cc_id, self.red, "E" + repr(detected_errors))

    def _above_rpm_threshold(self, rpm):
        """Check if rpm is above the rpm threshold to indicate spinning.

        Args:
            rpm (float): rpm
        """
        if abs(rpm) > self._RPM_THRESHOLD:
            return True
        return False

    def _set_cleaning_core_color_and_text(self, cc_id, color, text):
        """Set cleaning feedback color and text.

        Args:
            cc_id (int): cc_id
            color (ColorProperty): color
            text (string): text
        """
        if hasattr(self, "cc_" + str(cc_id) + "_color"):
            setattr(self, "cc_" + str(cc_id) + "_color", color)
        if hasattr(self, "cc_" + str(cc_id) + "_text"):
            setattr(self, "cc_" + str(cc_id) + "_text", text)

    def _is_timeout(self, time):
        """Check timeout between current time and msg recieved time.

        Returns:
            Bool: timeout flag
        """
        now = rospy.get_time()
        return (now - time) > self._timeout
