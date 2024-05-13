import math
import os
import sys

import kivy
import rospy
from mavros_msgs.msg import AttitudeTarget

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ObjectProperty
from kivy.uix.relativelayout import RelativeLayout
from nav_msgs.msg import Odometry

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("rollVisualiserPilotView.kv"))


class RollVisualiserPilotView(RelativeLayout):
    """
    Visualiser for bot roll shown as overlay in pilot view
    """

    roll_widget = ObjectProperty()

    def __init__(self, **kwargs):
        super(RollVisualiserPilotView, self).__init__(**kwargs)
        self.roll_widget.set_label("ROLL")

        # Subscribers
        self.odom_sub = rospy.Subscriber("~odometry", Odometry, callback=self.depth_cb)

    def depth_cb(self, odom):
        orientation = odom.pose.pose.orientation
        _, roll_deg = self.quat_to_pitch_roll(
            orientation.w, orientation.x, orientation.y, orientation.z
        )

        self.roll_widget.set_angle(round(roll_deg, 2))

    def quat_to_pitch_roll(self, w, x, y, z):
        # Converts a quaternion to pitch and roll
        # roll (y-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = -math.atan2(sinr_cosp, cosr_cosp) * (180 / math.pi)

        # pitch (x-axis rotation)
        sinp = 2 * (w * y - z * x)
        if math.fabs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) * (
                180 / math.pi
            )  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp) * (180 / math.pi)
        return pitch, roll
