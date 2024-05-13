import os

import kivy
import rospy
from std_msgs.msg import Int16

kivy.require("1.4.1")
from ButtonsControl import ButtonsControl
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("pointOfInterestPilotView.kv"))


class PointOfInterestPilotView(BoxLayout):
    """
    Button for logging a new point of interest in pilot view
    """

    POI_file = os.path.join(os.path.dirname(__file__), "../assets/POI.png")

    def __init__(self, **kwargs):
        super(PointOfInterestPilotView, self).__init__(**kwargs)

        # Controller instance for functionality
        self.buttons_controller = ButtonsControl.get_instance()

    def on_log_POI(self):
        self.buttons_controller.log_point_of_interest()
