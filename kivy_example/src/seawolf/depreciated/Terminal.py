import copy
import os
import sys
import time
from functools import partial

# Kivy
import kivy

kivy.require("1.2.0")

from kivy.app import App
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.properties import (
    BooleanProperty,
    DictProperty,
    NumericProperty,
    ObjectProperty,
    StringProperty,
)
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from kivy.uix.label import Label
from kivy.uix.widget import Widget

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("terminal.kv"))
# ROS
import rosnode
import rospy
import subs
from rosgraph_msgs.msg import Log

# Constants
DEBUG = 1
INFO = 2
WARN = 4
ERROR = 8
FATAL = 16
MAX_MESSAGES = 100
DELETE_SIZE = 20


class Terminal(Widget):
    message_list = DictProperty({})
    node_state = StringProperty("All Nodes")
    level_state = NumericProperty(3)
    auto_scroll = BooleanProperty(True)

    def __init__(self, **kwargs):
        super(Terminal, self).__init__(**kwargs)
        self.message_container = self.ids.message_container
        self.id_index = 0

        self.terminal_on = False

        # For scroll to work minimum height must be bound to height
        # (I don't really understand why, but it's in the docs)
        self.message_container.bind(minimum_height=self.message_container.setter("height"))

        rospy.Subscriber("/rosout", Log, self.create_message)

    def on_parent(self, widget, parent):
        # Here we bind the terminal's local state to the App's global state
        if parent:
            App.get_running_app().bind(level_filter=self.on_state_change)
            App.get_running_app().bind(node_filter=self.on_state_change)
            App.get_running_app().bind(terminal_switch=self.check_terminal_switch)

    def check_terminal_switch(self, instance, value):
        # Toggle terminal flag on app terminal switch property change (changed by setting panel)
        terminal_switch = App.get_running_app().terminal_switch
        if terminal_switch == "0":
            self.terminal_on = False
        elif terminal_switch == "1":
            self.terminal_on = True

    def on_state_change(self, instance, value):
        self.node_state = App.get_running_app().node_filter
        self.level_state = App.get_running_app().level_filter

    def create_message(self, data):
        # Only create message if terminal is on
        if self.terminal_on:
            # construct new message to add to terminal widget
            rosout = copy.deepcopy(data)
            new_message = Message(info=rosout)

            # We only want to add new messages:
            # To do this we compare stamps of the new and previous messages.
            # If the new message is not the first message, we grab the new and
            # previous headers so that we may compare.
            if self.id_index > 0:
                new_msg_stamp = getattr(new_message.info, "header").stamp
                prev_msg_stamp = getattr(self.message_list[self.id_index - 1].info, "header").stamp

            # New messages get added to the message_list
            if (self.id_index == 0) or (new_msg_stamp != prev_msg_stamp):
                Clock.schedule_once(partial(self.add_to_message_container, new_message))

    def add_to_message_container(self, msg, dt):
        self.message_list[self.id_index] = msg
        self.id_index += 1

        # Might (??) help with floating message glitch, but definitely doesn't
        # solve it completely
        time.sleep(0.01)

        # If msg is in domain of filter(s), add to message container
        node = getattr(msg, "info").name
        level = getattr(msg, "info").level
        if (self.node_state == "All Nodes" or self.node_state == node) and (
            level >= self.level_state
        ):
            self.message_container.add_widget(msg)
            if App.get_running_app().auto_scroll:
                self.ids.scrollview.scroll_to(msg)

        # Delete excess messages
        if len(self.message_list) >= MAX_MESSAGES:
            for x in range(DELETE_SIZE):
                message_index = self.id_index - MAX_MESSAGES + x
                self.message_container.remove_widget(self.message_list[message_index])
                del self.message_list[message_index]


class Message(Label):
    info = ObjectProperty()

    def __init__(self, **kwargs):
        super(Message, self).__init__(**kwargs)
        if self.info:
            severity = getattr(self.info, "level")
            if severity == DEBUG:
                level = "[DEBUG] "
                color = [1, 1, 1, 1]  # White
            elif severity == INFO:
                level = "[INFO] "
                color = [0, 1, 0, 1]  # Green
            elif severity == WARN:
                level = "[WARN] "
                color = [1, 1, 0, 1]  # Yellow
            elif severity == ERROR:
                level = "[ERROR] "
                color = [1, 0.5, 0, 1]  # Orange
            elif severity == FATAL:
                level = "[FATAL] "
                color = [1, 0, 0, 1]  # Red
            setattr(self, "text", level + getattr(self.info, "msg"))
            setattr(self, "color", color)


class Filter(BoxLayout):
    def __init__(self, **kwargs):
        super(Filter, self).__init__(**kwargs)

        self.node_filter_menu = DropDown(auto_width=False, width=150)
        self.level_filter_menu = DropDown(auto_width=False, width=150)

        # Generate Level filter Buttons
        level_btn_labels = [
            ">=1 [DEBUG]",
            ">=2 [INFO]",
            ">=3 [WARN]",
            ">=4 [ERROR]",
            ">=5 [FATAL]",
        ]

        def make_level_btn(label):
            btn = Button(
                text=label,
                height=30,
                size_hint_y=None,
                font_size=13,
                on_release=lambda btn: self.level_filter_menu.select(btn.text),
            )
            self.level_filter_menu.add_widget(btn)

        map(make_level_btn, level_btn_labels)

        # Change node/level button text to reflect current filter selection
        self.level_filter_menu.bind(on_select=self.handle_level_select)
        self.node_filter_menu.bind(on_select=self.handle_node_select)

        # Open dropdown when node/level button pressed
        # A work around since self.ids doesn't get filled until after init
        Clock.schedule_once(lambda _: self.ids.node_button.bind(on_release=self.open_node_list))
        Clock.schedule_once(
            lambda _: self.ids.level_button.bind(on_release=self.level_filter_menu.open)
        )
        Clock.schedule_once(lambda _: self.ids.autoscroll.bind(active=self.toggle_scroll))

    def open_node_list(self, widget):
        self.node_filter_menu.clear_widgets()

        # Generate Level filter Buttons
        node_btn_labels = ["All Nodes"] + rosnode.get_node_names()

        def make_node_btn(label):
            btn = Button(
                text=label,
                height=30,
                size_hint_y=None,
                font_size=13,
                on_release=lambda btn: self.node_filter_menu.select(btn.text),
            )
            self.node_filter_menu.add_widget(btn)

        map(make_node_btn, node_btn_labels)

        self.node_filter_menu.open(widget)

    def handle_node_select(self, instance, value):
        # Change state
        App.get_running_app().node_filter = value
        # Update label
        setattr(self.ids.node_button, "text", value.split("/")[-1])

    def handle_level_select(self, instance, value):
        level = int(value.split(" ")[0].split("=")[-1])
        level_enum = 2 ** (level - 1)
        # Change state
        App.get_running_app().level_filter = level_enum
        # Update label
        setattr(self.ids.level_button, "text", value)

    def toggle_scroll(self, instance, value):
        App.get_running_app().auto_scroll = value
