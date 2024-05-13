#!/usr/bin/env python
import os
import sys

import kivy
import roslaunch
import rospy

kivy.require("1.2.0")

from kivy.app import App
from kivy.graphics import PopMatrix, PushMatrix, Rectangle, Rotate
from kivy.lang import Builder
from kivy.properties import NumericProperty
from kivy.uix.relativelayout import RelativeLayout
from mavros_msgs.msg import VFR_HUD
from ReturnToHome import ReturnToHome

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("compass.kv"))


class Compass(RelativeLayout):
    # initializing needle angle variable.
    needle_angle = NumericProperty(0)
    rth_needle_angle = NumericProperty(0)

    # Asset file names.
    compass_rose_file = "../assets/rose_WB.png"
    compass_rose_path = os.path.join(os.path.dirname(__file__), compass_rose_file)
    compass_needle_file = "../assets/needle.png"
    compass_needle_path = os.path.join(os.path.dirname(__file__), compass_needle_file)

    def __init__(self, **kwargs):
        super(Compass, self).__init__(**kwargs)

        heading_sub = rospy.Subscriber(
            "/Hullbot/bridge/mav_sensor/vfr_hud", VFR_HUD, callback=self.heading_cb
        )

        # Draw the compass needle in Python
        # Doing it in kv works well for a few seconds but then seems to inexplicably stop
        with self.compass_layout.canvas.after:
            PushMatrix()
            self.compass_rot = Rotate()
            self.compass_rot.angle = self.needle_angle
            self.compass_rot.origin = self.center
            self.compass_img = Rectangle(source=self.compass_needle_path)
            self.compass_img.size = self.compass_layout.size
            self.compass_img.pos = [0, 0]
            PopMatrix()

        # Make it update on resize
        self.compass_layout.bind(pos=self.update_needle, size=self.update_needle)

    def heading_cb(self, vfr_hud):
        """
        Get the current heading from MAVLink
        heading is an int16 from 0..360
        """

        # Kivy rotates anticlockwise with increasing angle, the opposite of NED
        self.needle_angle = 360 - vfr_hud.heading

        self.update_needle()

    def update_needle(self, *args):
        """
        Update the needle drawing in the right position and orientation
        """
        self.compass_rot.angle = self.needle_angle
        self.compass_rot.origin = self.compass_layout.center
        self.compass_img.size = self.compass_layout.size

    def set_rth_heading(self, rth_heading):
        self.rth_needle_angle = 360 - rth_heading


if __name__ == "__main__":

    class WidgetApp(App):
        def build(self):
            return Compass()

    WidgetApp().run()
