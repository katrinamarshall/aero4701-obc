import os
import subprocess

import kivy

kivy.require("1.2.0")

import rosgraph
import rospy
from kivy.app import App
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.uix.popup import Popup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("noRoscorePopup.kv"))


class NoRoscorePopup(Popup):
    def __init__(self, **kwargs):
        super(NoRoscorePopup, self).__init__(**kwargs)

        feedback_str = "No roscore detected."
        feedback_str += "\nIs the bot on?\n"
        feedback_str += "\nROS_MASTER_URI is: " + rosgraph.rosenv.get_master_uri()
        our_ROS_IP = os.environ.get("ROS_IP", "")
        feedback_str += "\nROS_IP is: " + our_ROS_IP
        ip_addresses = subprocess.check_output(
            "ifconfig | grep 'inet ' | grep -v 127.0.0.1 | grep -v 172.* | awk '{print $2}'",
            shell=True,
        )
        feedback_str += "\nAnd our IPs are: " + ip_addresses
        # Check if our ROS_IP is in our IPs
        if our_ROS_IP != "" and our_ROS_IP.strip() not in ip_addresses:
            feedback_str += "\n\nWARNING: $ROS_IP is not any of our IP addresses."
        feedback_str += "\n\nWaiting for roscore"
        self.base_feedback_str = feedback_str
        self.moving_dots = ""
        self.error_label.text = self.base_feedback_str
        Clock.schedule_interval(self.check_roscore_and_update_popup, 0.5)

    # dt means delta-time
    def check_roscore_and_update_popup(self, dt):
        if rosgraph.is_master_online():
            # We stop this popup as it's run like an app before seawolf opens
            App.get_running_app().stop()
        else:
            if len(self.moving_dots) < 3:
                self.moving_dots += "."
            else:
                self.moving_dots = ""
            self.error_label.text = self.base_feedback_str + self.moving_dots


class NoRoscoreWidgetApp(App):
    def build(self):
        return NoRoscorePopup()


if __name__ == "__main__":
    NoRoscoreWidgetApp().run()
