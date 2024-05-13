import kivy
import rospy
from kivy.clock import Clock
from std_msgs.msg import Bool

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ColorProperty

import seawolf
from seawolf.utils.customWidgets import CustomTextIndicator


class ArmedIndicator(CustomTextIndicator):
    green = ColorProperty([0, 1, 0, 1])
    red = ColorProperty([1, 0, 0, 1])

    def __init__(self, **kwargs):
        super(ArmedIndicator, self).__init__(**kwargs)
        self.set_text("UNRESPONSIVE")
        self._last_time_stamp = 0
        self._timeout = 2
        self._arm_sub = rospy.Subscriber("~arm", Bool, callback=self._arm_cb)
        self._arm_msg = None

        update_rate = 5  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))

    def _arm_cb(self, msg):
        """Store ROS msg and recieved msg time.

        Args:
            msg (Bool): msg
        """
        self._last_time_stamp = rospy.get_time()
        self._arm_msg = msg

    def _update_graphics_cb(self, *args):
        """Update text and text color to either UNRESPONSIVE [white], ARMED [green] or DISARMED [red]."""
        if self._is_timeout():
            self.set_text("UNRESPONSIVE")
            self.set_neutral()
        elif self._arm_msg.data:
            self.set_text("ARMED")
            self.set_text_colour(self.green)
        elif not self._arm_msg.data:
            self.set_text("DISARMED")
            self.set_text_colour(self.red)

    def _is_timeout(self):
        """Check timeout between current time and msg recieved time.

        Returns:
            Bool: timeout flag
        """
        now = rospy.get_time()
        return (now - self._last_time_stamp) > self._timeout
