import os
from ast import List

import actionlib
import kivy
import rospy
from matplotlib.pyplot import connect

kivy.require("1.2.0")

from diagnostic_msgs.msg import DiagnosticArray
from hullbot_msgs.msg import (
    FeatherCommandAction,
    FeatherCommandGoal,
    FeatherCommandResult,
    FeatherStatus,
    PodWinchMotorStatus,
)
from kivy.lang import Builder
from kivy.properties import ListProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from std_msgs.msg import Float32

import seawolf
from seawolf.utils.ThreadingSupport import threaded

Builder.load_file(seawolf.form_kivy_config_path("podDisplayPane.kv"))

OK = 0
WARN = 1
ERROR = 2
STALE = 3


class PodDisplayPane(BoxLayout):
    attached_pod = StringProperty()
    mqtt_connected = StringProperty()
    serial_connected = StringProperty()

    torsion_spring_angle = StringProperty()
    internal_tether_length = StringProperty()
    reel_diameter = StringProperty()
    reel_speed = StringProperty()
    reel_torque = StringProperty()
    winch_speed = StringProperty()
    winch_torque = StringProperty()
    grey = ListProperty([1, 1, 1, 0])
    red = ListProperty([1, 0, 0, 0.5])
    green = ListProperty([0, 1, 0, 0.5])
    blue = ListProperty([0.17, 0.3, 1, 0.5])
    yellow = ListProperty([1, 0.6, 0, 0.5])

    pin_status_dict = {
        "BOT": "bot_enable_pin",
        "POD": "pod_enable_pin",
        "STACK": "stack_enable_pin",
        "SOL": "sol_enable_pin",
        "WINCH": "winch_enable_pin",
        "CHARGE": "charge_enable_pin",
    }

    def __init__(self, **kwargs):

        super(PodDisplayPane, self).__init__(**kwargs)

        self.spring_sub = rospy.Subscriber(
            "/hullbot/pod/torsion_spring_angle", Float32, callback=self.spring_callback
        )
        self.internal_tether_sub = rospy.Subscriber(
            "/hullbot/pod/tether_inside", Float32, callback=self.tether_callback
        )
        self.reel_diameter_sub = rospy.Subscriber(
            "/hullbot/pod/reel_diameter", Float32, callback=self.reel_diameter_callback
        )
        self.reel_status_sub = rospy.Subscriber(
            "/hullbot/pod_motor/reel/status",
            PodWinchMotorStatus,
            callback=self.reel_status_callback,
        )
        self.winch_status_sub = rospy.Subscriber(
            "/hullbot/pod_motor/winch/status",
            PodWinchMotorStatus,
            callback=self.winch_status_callback,
        )
        self.feather_diagnostics_sub = rospy.Subscriber(
            "/diagnostics", DiagnosticArray, callback=self.feather_diagnostics_callback
        )
        self.feather_pins_status_sub = rospy.Subscriber(
            "/hullbot/pod/feather/response/status",
            FeatherStatus,
            callback=self.feather_status_callback,
        )
        self.feather_action_client = actionlib.SimpleActionClient(
            "feather_command", FeatherCommandAction
        )
        self.feather_labels = {}

        # Generate feather pin labels
        for pin_key in sorted(self.pin_status_dict.keys()):
            pin_label = Label(text=pin_key, bold=True)
            self.feather_labels[self.pin_status_dict[pin_key]] = pin_label
            self.ids.feather_pins_status.add_widget(pin_label)

    def spring_callback(self, angle):
        self.torsion_spring_angle = str(round(angle.data, 2)) + " deg"

    def tether_callback(self, length):
        self.internal_tether_length = str(round(length.data, 2)) + " m"

    def reel_diameter_callback(self, diameter):
        self.reel_diameter = str(round(diameter.data, 2)) + " m"

    def reel_status_callback(self, reel_status):
        self.reel_speed = str(round(reel_status.speed, 2)) + " m/s"
        self.reel_torque = str(round(reel_status.torque, 2)) + " Nm"

    def winch_status_callback(self, winch_status):
        self.winch_speed = str(round(winch_status.speed, 2)) + " m/s"
        self.winch_torque = str(round(winch_status.torque, 2)) + " Nm"

    def feather_status_callback(self, feather_status):
        """
        Update feather status
        """
        # Pod name
        self.attached_pod = getattr(feather_status, "attached_pod")

        # Feather pins
        for status_key in self.feather_labels.keys():
            status = getattr(feather_status, status_key)
            if status:
                self.feather_labels[status_key].color = self.green
            else:
                self.feather_labels[status_key].color = self.red
                pass

        # Mqtt status
        mqtt_status = getattr(feather_status, "mqtt_connected")
        self.mqtt_connected = self.mqtt_connection_conversion(mqtt_status)

        # Serial status
        if getattr(feather_status, "serial_connected"):
            self.serial_connected = "Connected"
        else:
            self.serial_connected = "Disconnected"

    def mqtt_connection_conversion(self, connection):
        """
        Converts binary representation of mqtt connection stages to human-readable format
        """

        mqtt_connection_dict = {
            0b00000001: "CERTFICATE",
            0b00000011: "MODEM",
            0b00000111: "CLIENT",
            0b00001111: "FD",
            0b00011111: "CONNECT",
            0b00111111: "POLL",
        }

        if connection & 0b01000000:
            return "ERROR"

        return mqtt_connection_dict.get(connection, str(connection))

    def feather_diagnostics_callback(self, msg):
        """
        Parses diagnostics messsage to find feather watchdog status
        """
        statuses = msg.status
        for message in statuses:
            name = message.name

            if name == "Feather Comms Watchdog":
                if message.level == OK:
                    self.ids.feather_node_status.text = "FEATHER STATUS: OK"
                    self.ids.feather_node_status.background_color = self.blue
                elif message.level == WARN:
                    self.ids.feather_node_status.text = "FEATHER STATUS: WARN"
                    self.ids.feather_node_status.background_color = self.yellow
                elif message.level == ERROR:
                    self.ids.feather_node_status.text = "FEATHER STATUS: ERROR"
                    self.ids.feather_node_status.background_color = self.red
                elif message.level == STALE:
                    self.ids.feather_node_status.text = "FEATHER STATUS: STALE"
                    self.ids.feather_node_status.background_color = self.grey
                else:
                    print("Unexpected behaviour with {}".format(name))
