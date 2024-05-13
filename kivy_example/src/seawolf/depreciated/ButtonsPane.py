import os
import subprocess
import time

import kivy
import rospy
from std_srvs.srv import Trigger

kivy.require("1.2.0")
from ButtonsControl import ButtonsControl
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ListProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label

import seawolf
from seawolf.utils.ThreadingSupport import *

Builder.load_file(seawolf.form_kivy_config_path("buttonsPane.kv"))


class SupervisorIndicator(Label):
    statusColour = ListProperty([1, 0, 0, 0.5])  # set red by default

    def __init__(self, **kwargs):
        super(SupervisorIndicator, self).__init__(**kwargs)

    def alive(self):
        self.statusColour = [0.17, 0.3, 1, 0.5]  # set blue

    def dead(self):
        self.statusColour = [1, 1, 1, 0.4]  # set grey


class ButtonsPane(BoxLayout):
    launch_topside_button_str = StringProperty()

    def __init__(self, **kwargs):
        super(ButtonsPane, self).__init__(**kwargs)

        # Instance to provide overall control
        self.buttons_controller = ButtonsControl.get_instance()

        self.launch_topside_button_str = "Launch Topside"
        if self.buttons_controller.is_topside_local:
            self.launch_topside_button_str += " (Laptop)"
        else:
            self.launch_topside_button_str += " (Pod)"

        self.topside_supervisor_monitor_timer = rospy.Timer(
            rospy.Duration(1.0), self.topside_supervisor_server_monitor
        )

    def topside_supervisor_server_monitor(self, _):
        is_alive = self.is_supervisor_server_up(self.topside_proc_server)
        if is_alive:
            self.ids.supervisor_indicator.alive()
        else:
            self.ids.supervisor_indicator.dead()

    def launch_rviz(self):
        self.buttons_controller.launch_rviz()

    def launch_rqt(self):
        self.buttons_controller.launch_rqt()

    def launch_plotjuggler(self):
        self.buttons_controller.launch_plotjuggler()

    def launch_topside(self):
        self.buttons_controller.launch_topside()

    def run_topside_procs(self):
        self.buttons_controller.run_topside_procs()

    def start_slam_common(self):
        self.buttons_controller.start_slam_common()

    def launch_tx2_core(self):
        self.buttons_controller.launch_botside()

    def kill_topside(self):
        self.buttons_controller.kill_topside()

    def kill_botside(self):
        self.buttons_controller.kill_botside()

    def rebootPixhawk(self):
        self.buttons_controller.rebootPixhawk()

    def open_rosbag_popup(self):
        self.buttons_controller.open_rosbag_popup()

    def log_point_of_interest(self):
        self.buttons_controller.log_point_of_interest()

    def retrieve_remote_logs(self):
        self.buttons_controller.retrieve_remote_logs()

    def on_stop(self):
        self.buttons_controller.on_stop()


if __name__ == "__main__":

    class WidgetApp(App):
        def build(self):
            return ButtonsPane()

    rospy.init_node("buttons_pane_test")
    WidgetApp().run()
