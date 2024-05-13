# Shows a graphical display of the brush cleaning status (on/off)

import os

import kivy
import rospy
from std_msgs.msg import Int16

kivy.require("1.4.1")
from hullbot_msgs.msg import VescStatusArray
from kivy.lang import Builder
from kivy.properties import StringProperty
from kivy.uix.boxlayout import BoxLayout

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("brushStatePilotView.kv"))


class BrushStatePilotView(BoxLayout):
    "Indicator for whether the CCs are running and their duty cycle when in pilot view"
    duty_cycle = StringProperty("0.0")

    # Minimum duty cycle for cleaning core to be classified as 'on'
    ON_DUTY_THRESHOLD = 0.05
    # Minimum rpm for cleaning core to be classified as 'on'
    ON_RPM_THRESHOLD = 500

    def __init__(self, **kwargs):
        super(BrushStatePilotView, self).__init__(**kwargs)
        self.cc_feedback_sub = rospy.Subscriber(
            "/hullbot/cleaning_cores/feedback", VescStatusArray, callback=self.cc_feedback_cb
        )
        self.ids.dc_overlay.set_background_color(0, 0, 0, 1)

    def cc_feedback_cb(self, msg):
        """
        Callback for cleaning core feedback (rpm, duty cycle)
        """
        for status in msg.status:
            if (
                abs(status.duty_cycle) >= self.ON_DUTY_THRESHOLD
                or abs(status.rpm) >= self.ON_RPM_THRESHOLD
            ):
                self.duty_cycle = str(round(status.duty_cycle, 2))
