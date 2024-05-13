import math
import os
import sys

import kivy
import rospy
from kivy.clock import Clock
from nav_msgs.msg import Odometry

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import NumericProperty
from kivy.uix.image import Image
from kivy.uix.relativelayout import RelativeLayout

import seawolf
from seawolf import quat_to_pitch_roll

Builder.load_file(seawolf.form_kivy_config_path("rollIndicator.kv"))


class RollIndicator(RelativeLayout):
    """
    Visualiser for bot roll
    """

    bot_graphic_file = "back_view.png"

    def __init__(self, **kwargs):
        # Asset file names
        self.bot_graphic_path = seawolf.form_assets_path(self.bot_graphic_file)

        # Initialise
        super(RollIndicator, self).__init__(**kwargs)

        # Subscribers for functionality
        self.odom_sub = rospy.Subscriber("~odometry", Odometry, callback=self.odom_cb)

        update_rate = 10  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))
        self.roll_deg = 0.0

    def odom_cb(self, odom):
        orientation = odom.pose.pose.orientation
        _, self.roll_deg = quat_to_pitch_roll(
            orientation.w, orientation.x, orientation.y, orientation.z
        )

    def _update_graphics_cb(self, *args):
        self.ids.indicator_image.setAngle(round(self.roll_deg, 2))


class RotatableImage(Image):
    angle = NumericProperty(0)

    @property
    def getAngle(self):
        return self.angle

    def setAngle(self, angle):
        self.angle = angle
