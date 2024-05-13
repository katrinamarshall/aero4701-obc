import os
from ast import Num

import kivy

kivy.require("1.4.1")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import NumericProperty
from kivy.uix.behaviors import ButtonBehavior
from kivy.uix.boxlayout import BoxLayout
from TofDisplay import TOFErrorPopup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("tofDisplayPilotView.kv"))


import rospy
from hullbot_msgs.msg import TOFChain


class TofDisplayPilotView(BoxLayout, ButtonBehavior):
    """
    Display of front TOF distances in pilot view
    """

    top_front_star = NumericProperty(160)
    top_back_star = NumericProperty(160)
    top_front_port = NumericProperty(160)
    top_back_port = NumericProperty(160)
    front_star = NumericProperty(160)
    front_port = NumericProperty(160)
    back_star = NumericProperty(160)
    back_port = NumericProperty(160)

    def __init__(self, **kwargs):
        super(TofDisplayPilotView, self).__init__(**kwargs)
        self.tof_sub = rospy.Subscriber(
            "/hullbot/tof_node/all_tofs_filtered", TOFChain, self.tof_cb
        )

        # Initialise the error popup to be shown when errors occur
        self.tof_mode_popup = TOFErrorPopup()

    def tof_cb(self, tof_data):
        """
        Callback for new TOF data from ROS
        """
        # Get the new data and convert from m to mm
        self.top_front_star = tof_data.ranges[1] * 1000
        self.top_back_star = tof_data.ranges[2] * 1000
        self.top_front_port = tof_data.ranges[4] * 1000
        self.top_back_port = tof_data.ranges[5] * 1000

        self.front_star = tof_data.ranges[1] * 1000
        self.front_port = tof_data.ranges[2] * 1000
        self.back_star = tof_data.ranges[4] * 1000
        self.back_port = tof_data.ranges[5] * 1000
