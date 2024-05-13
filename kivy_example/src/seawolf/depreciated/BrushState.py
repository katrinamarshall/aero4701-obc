# Shows a graphical display of the brush cleaning status (on/off)

import os

import kivy
import rospy
from std_msgs.msg import Int16

kivy.require("1.4.1")
from hullbot_msgs.msg import VescStatusArray
from kivy.lang import Builder
from kivy.properties import BooleanProperty, ListProperty
from kivy.uix.image import Image
from kivy.uix.widget import Widget

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("brushstate.kv"))


class BrushState(Image):
    """
    This class provides an image widget which can be added into a UI to display the
    on/off state of the brushes
    """

    # Minimum duty cycle for cleaning core to be classified as 'on'
    ON_DUTY_THRESHOLD = 0.05
    # Minimum rpm for cleaning core to be classified as 'on'
    ON_RPM_THRESHOLD = 500

    cleaning_status = BooleanProperty(False)
    color_on = ListProperty([0, 1, 0, 1])
    color_off = ListProperty([1, 1, 1, 0.4])

    def __init__(self, **kwargs):
        # Asset file names
        brush_file = "../assets/brush.png"
        self.brush_path = os.path.join(os.path.dirname(__file__), brush_file)

        try:
            super(BrushState, self).__init__(**kwargs)
        except Exception as e:
            print(e)

        self.cc_feedback_sub = rospy.Subscriber(
            "/hullbot/cleaning_cores/feedback", VescStatusArray, callback=self.cc_feedback_cb
        )
        self.color = self.color_off

    def cc_feedback_cb(self, msg):
        """
        Callback for cleaning core feedback (rpm, duty cycle)
        """
        brush_cleaning_status = False
        # Check all brushes are on
        for status in msg.status:
            if (
                abs(status.duty_cycle) >= self.ON_DUTY_THRESHOLD
                or abs(status.rpm) >= self.ON_RPM_THRESHOLD
            ):
                brush_cleaning_status = True
                break

        # Only update if cleaning status has changed
        if brush_cleaning_status != self.cleaning_status:
            self.cleaning_status = brush_cleaning_status
            self.update_display()

    def update_display(self):
        """
        Update display to indicate whether cleaning is happening
        """
        if self.cleaning_status:
            self.color = self.color_on
        else:
            self.color = self.color_off
