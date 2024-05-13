import os
import sys
from functools import partial

import kivy
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger

kivy.require("1.2.0")
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.properties import NumericProperty, StringProperty
from kivy.uix.button import Button
from kivy.uix.relativelayout import RelativeLayout

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("calibrationpane.kv"))

import pubs
import subs

# modes
WAITING = 0
ACCELEROMETER = 1
PRESSURE = 2
instructions = [
    # waiting
    [
        "Select a Calibration Routine to begin",
        "Calibration failed, try again.",
        "Select a Calibration Routine to begin \nCould not begin calibration",
        "Could not calibrate barometer. Please try again.",
    ],
    # accelerometer
    [
        "Place the vehicle level\nand click 'Next' to continue",
        "Place the vehicle on its LEFT side\nand click 'Next' to continue",
        "Place the vehicle on its RIGHT and\nclick 'Next' to continue",
        "Place the vehicle nose DOWN and\nclick 'Next' to continue",
        "Place the vehicle nose UP and\nclick 'Next' to continue",
        "Place the vehicle on its BACK\nand click 'Next' to continue",
        "Calibration Complete!",
    ],
    # pressure
    [
        "Place the vehicle just below the water's surface\nand click 'Next' to continue",
        "Calibration Complete!",
    ],
]


class NextButton(Button):
    pass


class CalibrationPane(RelativeLayout):
    mode = NumericProperty(WAITING)
    phase = NumericProperty(0)
    currentInstruction = StringProperty(instructions[0][0])

    def __init__(self):
        super(CalibrationPane, self).__init__()
        self.calibrate_accel = rospy.ServiceProxy("trigger_accel_cal", Trigger)
        self.calibrate_baro = rospy.ServiceProxy("trigger_baro_cal", Trigger)
        self.send_ack = rospy.ServiceProxy("trigger_send_ack", Trigger)
        self.sub = subs.Subs()
        self.sub.subscribe_topic("/Hullbot/pixhawk/calibration_status", String)

    def getCurrentInstruction(self):
        return instructions[self.mode, self.phase]

    def start_accel(self):

        # Send message
        try:
            # Remove/spawn Buttons as needed
            self.calibrate_accel()
            self.remove_widget(self.ids.acc_button)
            self.remove_widget(self.ids.pres_button)
            self.next = NextButton()
            self.add_widget(self.next)
            # Change state params
            self.mode = ACCELEROMETER
            self.phase = 0
            self.currentInstruction = instructions[self.mode][self.phase]
            # Set clock to poll for and update user feedback
            self.response_clock = Clock.schedule_interval(
                partial(self.await_response, "level"), 0.1
            )
            # #disable next button until response
            self.next.disabled = True
        except rospy.ServiceException as e:
            print("trigger_accel_cal service did not process request: " + str(e))
            self.phase = 2
            self.currentInstruction = instructions[self.mode][self.phase]

    def start_baro(self):
        # Remove/spawn Buttons as needed
        self.remove_widget(self.ids.acc_button)
        self.remove_widget(self.ids.pres_button)
        self.next = NextButton()
        self.add_widget(self.next)
        # Change state params
        self.mode = PRESSURE
        self.currentInstruction = instructions[self.mode][self.phase]

    def reset_buttons(self):
        self.remove_widget(self.next)
        self.add_widget(self.ids.acc_button)
        self.add_widget(self.ids.pres_button)

    def await_response(self, word, _):
        try:
            feedback = self.sub.get_data()["Hullbot"]["pixhawk"]["calibration_status"]["data"]
            # print word, feedback #DEBUG
            if word in feedback:
                Clock.unschedule(self.response_clock)
                self.next.disabled = False
                self.currentInstruction = instructions[self.mode][self.phase]
                # Reset if last phase in routine
                if self.phase == 6:
                    self.phase = 0
                    self.mode = WAITING
                    self.reset_buttons()
            elif "FAILED" in feedback:
                self.phase = 1
                self.mode = WAITING
                self.reset_buttons()
                self.currentInstruction = instructions[self.mode][self.phase]
                Clock.unschedule(self.response_clock)

        except:
            print("No messages received from bridge")

    def handle_next(self):
        self.next.disabled = True
        keywords = ["LEFT", "RIGHT", "DOWN", "UP", "BACK", "success"]
        if self.mode == ACCELEROMETER:
            if self.phase <= 5:
                self.response_clock = Clock.schedule_interval(
                    partial(self.await_response, keywords[self.phase]), 0.1
                )
                try:
                    self.send_ack()
                except rospy.ServiceException as e:
                    print("trigger_send_ack service did not process request: " + str(e))
                self.phase += 1
            elif self.phase == 6:
                self.phase = 0
                self.mode = WAITING
                self.reset_buttons()
        elif self.mode == PRESSURE:
            if self.phase == 0:
                try:
                    self.calibrate_baro()
                    self.response_clock = Clock.schedule_interval(
                        partial(self.await_response, "calibration complete"), 0.1
                    )
                    self.phase = 1
                except rospy.ServiceException as e:
                    self.phase = 3
                    self.mode = WAITING
                    self.next.disabled = False
                    print("trigger_baro_cal service did not process request: " + str(e))
            elif self.phase == 1:
                self.phase = 0
                self.mode = WAITING
                self.reset_buttons()
        elif self.mode == WAITING:
            self.phase = 0
            self.reset_buttons()

        self.currentInstruction = instructions[self.mode][self.phase]
