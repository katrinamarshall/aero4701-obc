import kivy
import rospy
from hb_localisation_msgs.msg import FrontendFeedback
from kivy.clock import Clock

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ColorProperty, ListProperty, StringProperty
from kivy.uix.image import Image
from kivy.uix.relativelayout import RelativeLayout

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("visionIndicator.kv"))


class VisionIndicator(Image):
    green = ColorProperty([0, 0.6, 0, 1])
    blue = ColorProperty([0.17, 0.3, 1, 1])
    grey = ColorProperty([1, 1, 1, 0.5])
    color = ColorProperty([1, 1, 1, 0.5])  # Property of Image

    vision_graphic_file = "robot-eye.png"

    def __init__(self, **kwargs):
        self.vision_graphic_path = seawolf.form_assets_path(self.vision_graphic_file)

        try:
            super(VisionIndicator, self).__init__(**kwargs)
        except Exception as e:
            print(e)

        self._timeout = 1  # s
        self._last_time_stamp = 0
        self._frontend_feedback_sub = rospy.Subscriber(
            "~frontend_feedback", FrontendFeedback, callback=self._feedback_cb
        )
        self._frontend_msg = FrontendFeedback()

        update_rate = 1  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))

    def _set_colour(self, colour):
        """Sets indicator colour."""
        self.color = colour

    def _feedback_cb(self, msg):
        """Store feedback msg and recieved msg time.

        Args:
            msg (FrontendFeedback): msg
        """
        self._last_time_stamp = rospy.get_time()
        self._feedback_msg = msg

    def _update_graphics_cb(self, *args):
        """Update vision indicator to either UNRESPONSIVE [grey], ENABLED [green] or DISABLED [blue]."""
        if self._is_timeout():
            self._set_colour(self.grey)
        elif self._frontend_msg.vision_enabled:
            self._set_colour(self.green)
        elif not self._frontend_msg.vision_enabled:
            self._set_colour(self.blue)

    def _is_timeout(self):
        """Check timeout between current time and msg recieved time.

        Returns:
            Bool: timeout flag
        """
        now = rospy.get_time()
        return (now - self._last_time_stamp) > self._timeout
