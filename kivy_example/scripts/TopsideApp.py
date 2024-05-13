#!/usr/bin/env python
import os
import platform
import shlex
import subprocess
import sys
import traceback

import kivy
import rosgraph
import rosnode
import rospy

import seawolf

kivy.require("1.2.0")
import signal

from kivy.app import App
from kivy.base import ExceptionManager
from kivy.clock import Clock
from kivy.config import ConfigParser
from kivy.core.text import LabelBase
from kivy.core.window import Window
from kivy.lang import Builder
from kivy.properties import BooleanProperty, NumericProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout

from src.seawolf.seawolf_settings import seawolf_settings_json
from src.seawolf.utils.ThreadingSupport import ThreadedExceptionHandler
from src.seawolf.view.demo_view import DemoView
from src.seawolf.view.full_screen import FullScreen
from src.seawolf.view.pilot_view import PilotView
from src.seawolf.widgets.ButtonsControl import ButtonsControl
from src.seawolf.widgets.Keypress import KeyHandler

Builder.load_file(seawolf.form_kivy_config_path("topside.kv"))


class TopsideApp(App):
    icon = seawolf.form_assets_path("hullbot_logo_small.png")
    title = "hullbot"
    ### Global state ###
    # Terminal Widget #
    node_filter = StringProperty("All Nodes")
    level_filter = NumericProperty(1)
    auto_scroll = BooleanProperty(True)
    demo_view = StringProperty()
    terminal_switch = StringProperty()

    def build(self):
        self.seawolf = SeawolfWindow()
        # For debugging uncomment the 2 lines below to enable the kivy inspector, then press Ctrl-E in kivy to inspect
        from kivy.modules import inspector

        inspector.create_inspector(Window, self.seawolf)
        return self.seawolf

    def build_config(self, config):
        self.seawolf_config = ConfigParser()
        self.seawolf_config.setdefaults(
            "Seawolf",
            {
                "demo_view": "0",  # default dual screen view
                "terminal_switch": "0",  # default terminal off
                "numericexample": 39,
                "optionsexample": "option2",
                "stringexample": "some_string",
                "pathexample": "/home/",
            },
        )
        # The sections cant contain underscores... and spaces are cumbersome
        config.setdefaults(
            "previousmission", {"previousmissionname": "", "previousmissionrobot": ""}
        )

    def build_settings(self, settings):
        # Add config selections defined by json
        settings.add_json_panel("Seawolf", self.seawolf_config, data=seawolf_settings_json)

    def on_config_change(self, config, section, key, value):
        # store new settings into corresponding property
        if key == "demo_view":
            self.demo_view = value
        elif key == "terminal_switch":
            self.terminal_switch = value

    def on_start(self):
        pass

    def on_stop(self):
        """Kill topside processes on exit as well as stop rosbag recording."""
        buttons_control = ButtonsControl()
        buttons_control.kill_topside_on_exit()


# The base class contains the topside layouts (pilot view or dual screen)
class SeawolfWindow(BoxLayout):
    def __init__(self, **kwargs):
        super(SeawolfWindow, self).__init__(**kwargs)
        self.demo_view = DemoView()
        self.pilot_view = PilotView()
        self.full_screen = FullScreen()
        self.ids.everything.add_widget(self.full_screen)

        # Set default view to Pilot View
        self.full_screen.set_view_type(self.pilot_view)
        # As follows, unsubscribe all demo view topics
        self.demo_view.close_video_streams()
        self.key_handler = KeyHandler()
        Window.bind(on_key_down=self.key_handler.key_down)
        Window.bind(on_key_up=self.key_handler.key_up)

    def on_parent(self, widget, parent):
        if parent:
            App.get_running_app().bind(demo_view=self.check_demo_view)

    def check_demo_view(self, instance, value):
        demo_view_on = App.get_running_app().demo_view

        # Demo view
        if demo_view_on == "1":
            self.pilot_view.close_video_streams()
            self.full_screen.set_view_type(self.demo_view)
            self.demo_view.open_video_streams()
        # Pilot view
        else:
            self.demo_view.close_video_streams()
            self.full_screen.set_view_type(self.pilot_view)
            self.pilot_view.open_video_streams()


def shutdown_signal(frame, sig):
    """[Shutdown function called when SIGINT called]

    Args:
        frame ([type]): [description]
        sig ([type]): [description]
    """
    print("Ctrl-C signal recieved. Shutting down ROS and Kivy")

    # Stop the app. This calls the on_stop method to ensure that the bagfiles stop.
    App.get_running_app().stop()

    # Shutdown ROS becuase we overwride the inbuilt signal handler.
    rospy.signal_shutdown("User shutdown")
    sys.exit(1)


if __name__ == "__main__":
    # First thing we need to do is check if there is a roscore
    # If there isn't show a popup warning of it and giving some info (see NoRoscorePopup.py)
    # This way we don't just show a black window if something is wrong when starting seawolf
    if not rosgraph.is_master_online():
        print("\n\n!!!!  No roscore detected.       !!!!")

        # Launch the 'No roscore found' popup as a separate process as we can't open two Kivy windows in one app
        # This is way easier than dealing with a ScreenManager
        # Note that we are opening a window on top of a black window, apparently, on import of
        # 'from kivy.core.window import Window' Kivy creates a OpenGL window. No way around it without
        # really ugly hacks (like doing lazy-importing using importlib)
        import os

        # Block until window that says no roscore is found closes (cause there is roscore) or is closed
        os.system("python /home/user/catkin_ws/src/seawolf/widgets/NoRoscorePopup.py")

    try:
        # Allow to have more than one seawolf pointing to a robot (useful for test panel)
        # possibly useful for when we use wifi on the boat
        default_node_name = "seawolf"
        node_names = rosnode.get_node_names()
        if default_node_name in node_names:
            rospy.logwarn(
                "Another seawolf is open with the rosnode name: {}.".format(default_node_name)
            )
            # Adds name of the machine
            default_node_name += "_" + platform.node()

        # Disable signals so we can use our own handler
        rospy.init_node(default_node_name, disable_signals=True)
        signal.signal(signal.SIGINT, shutdown_signal)

        LabelBase.register(name="formular", fn_regular=seawolf.get_assets_path() + "formular.ttf")
        LabelBase.register(
            name="comic_sans", fn_regular=seawolf.get_assets_path() + "LDFComicSansBold.ttf"
        )

        # Register the exception handler thread so that seawolf will stop swallowing errors and exit
        # when something is raised in a thread
        ExceptionManager.add_handler(ThreadedExceptionHandler())
        # Start the image relays when we are attached to a roscore
        TopsideApp().run()
    # Show all kivy exceptions
    except Exception as e:
        # If the exception that was raised came from a threaded decorator function then print that and
        # not the main traceback call as the information won't be there
        if ThreadedExceptionHandler.has_threaded_traceback():
            ThreadedExceptionHandler.print_tb()
        else:
            traceback.print_exc(file=sys.stdout)
    sys.exit(1)
