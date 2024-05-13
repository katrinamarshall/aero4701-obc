#!/usr/bin/env python
import math
import os.path

import kivy
import rospy
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.srv import SetMode, SetModeRequest
from std_msgs.msg import Float32MultiArray

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import NumericProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("ReturnToHome.kv"))


class ReturnToHome(BoxLayout):
    """
    This class provides functionality for returning the bot to home. The user sets a certain heading towards home, and
    a safe depth at which to return. When the bot gets lost, the "Return To Home" button can be pressed, causing the
    bot to orient itself to the home heading and sink to the safe depth set. The user can then simply move the bot
    forwards until they see it in the water or manage to reorient themselves.
    """

    # Default safe depth
    home_depth = NumericProperty(-2)

    def __init__(self, compass_obj, **kwargs):
        """
        Passing in the compass_obj allows us to display the blue tick to show RTH direction on the compass
        """
        super(ReturnToHome, self).__init__(**kwargs)

        self.compass = compass_obj
        self.depth_set_popup = DepthSetPopup()

        # Initialise variables
        self.home_heading = float(0)
        self.depth = None
        self.heading = None

        # Initialise callback for heading and depth
        self.hud_sub = rospy.Subscriber(
            "/Hullbot/bridge/mav_sensor/vfr_hud", VFR_HUD, callback=self.hud_cb
        )

        self.set_control_mode_client = rospy.ServiceProxy(
            "/hullbot/xyz_control/set_control_mode", SetMode
        )
        self.set_attitude_eul_pub = rospy.Publisher(
            "/Hullbot/bridge/attitude/set_eul", Vector3, queue_size=1
        )

        self.depth_target_pub = rospy.Publisher(
            "/hullbot/xyz_control/depth_reference", Float32MultiArray, queue_size=1
        )

        self.dest_angle_msg = Vector3()
        self.dest_angle_msg.x = 0
        self.dest_angle_msg.y = 0

        self.dest_depth_msg = Float32MultiArray()

    def hud_cb(self, vfr_hud):
        """
        Collect the current heading and depth
        """
        self.heading = vfr_hud.heading
        self.depth = vfr_hud.altitude

    def set_home_heading_on_release(self):
        """
        Callback for the Set Heading button
        """
        if self.heading is not None:
            self.home_heading = self.heading
            self.compass.set_rth_heading(self.home_heading)

            # Set the button back to default colour (no longer red)
            self.heading_button.background_color = (1, 1, 1, 1)

    def set_home_depth_on_release(self):
        """
        Callback for the Set Depth button, opens a popup to allow user to set depth
        """
        self.depth_set_popup.open(str(round(self.depth, 3)), self.set_home_depth)

    def set_home_depth(self, depth):
        """
        Called by the popup with the final depth value to set
        """
        self.home_depth = depth
        # Set the button back to default colour (no longer red)
        self.depth_button.background_color = (1, 1, 1, 1)

    def rotateToHome(self):
        """
        Rotate the bot towards home and make it flat
        """

        try:
            # Set DEPTH_HOLD mode
            control_mode_req = SetModeRequest()
            control_mode_req.custom_mode = "DEPTH_HOLD"
            resp = self.set_control_mode_client(control_mode_req)
        except rospy.ServiceException as e:
            popup = Popup(
                title="Return To Home aborted",
                content=Label(text="RTH: Couldn't change to depth hold mode\n" + repr(e)),
                size_hint=(None, None),
                size=(600, 200),
            )
            popup.open()

        # Set yaw (in radians)
        self.dest_angle_msg.z = math.radians(self.home_heading)

        # Set target depth
        self.dest_depth_msg.data = [0, 0, self.home_depth, 0, 0, 0]

        # Publish the destination attitude and depth
        self.set_attitude_eul_pub.publish(self.dest_angle_msg)
        # TODO: Fix depth hold interaction with manual.py which overrides the depth goal set here:
        # self.depth_target_pub.publish(self.dest_depth_msg)


class DepthSetPopup(Popup):
    """
    Popup to allow the user to set a custom RTH depth
    Means that we don't actually have to swim the bot down to that depth to set it as this may be invonvenient
    The initial value shown is the current depth of the bot
    """

    # Disallow depths outside this range
    min_depth = NumericProperty(-20)
    max_depth = NumericProperty(0)

    def __init__(self, **kwargs):
        super(DepthSetPopup, self).__init__(**kwargs)
        self.prev_str = ""

    def open(self, current_depth, set_depth_func):
        self.set_depth_func = set_depth_func
        self.depth_text_input.text = current_depth

        super(DepthSetPopup, self).open()

    def on_depth_text_input(self):
        """
        Enforce valid number input for depth
        """

        # Force the user to only input a valid float
        try:
            float(self.depth_text_input.text)
            self.prev_str = self.depth_text_input.text
        except:
            self.depth_text_input.text = self.prev_str

    def set_depth_on_release(self):
        """
        Callback for the set depth button
        """
        depth = float(self.depth_text_input.text)

        # Make sure the depth is in a valid range
        if self.min_depth < depth < self.max_depth:
            # Call RTH set_home_depth
            self.set_depth_func(float(self.depth_text_input.text))
            self.dismiss()
        else:
            self.error_label.text = "Depth must be between {} and {}".format(
                self.min_depth, self.max_depth
            )
