import atexit
import os
from concurrent.futures import thread

import kivy
import rospy
from ButtonsControl import PodControl
from std_msgs.msg import Bool, Float32

kivy.require("1.2.0")

from kivy.lang import Builder
from kivy.properties import ObjectProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from kivy.uix.gridlayout import GridLayout
from kivy.uix.spinner import Spinner

import seawolf
from seawolf.utils.ThreadingSupport import threaded

Builder.load_file(seawolf.form_kivy_config_path("podControlPane.kv"))


class PodCommandDropDown(GridLayout):

    # See configuration in
    # https://www.notion.so/hullbot/Feather-Communication-Protocol-220bb59e3ddc47b088a8a1a3c2cec5f4
    pin_name_dict = {"CHARGE": 8, "BOT": 7, "SOL": 6, "WINCH": 5, "STACK": 4, "POD": 3}

    def __init__(self, **kwargs):
        super(PodCommandDropDown, self).__init__(cols=2)
        self.dropdown = DropDown()
        self.selection_id = "-"
        self.confirm_cb = None

        spinner_buttons = []
        for index in self.pin_name_dict.keys():
            pin_type = index
            pin_type_enable = pin_type + "_ON"
            pin_type_disable = pin_type + "_OFF"
            spinner_buttons.append(pin_type_enable)
            spinner_buttons.append(pin_type_disable)

        self.spinner = Spinner(
            text="-", values=tuple(spinner_buttons), pos_hint={"center_x": 0.5, "center_y": 0.5}
        )
        self.spinner.bind(text=self.command_selection_cb)
        self.add_widget(self.spinner)

        send_command_button = Button(text="Confirm", size_hint=(0.5, 0.5), height=15)
        send_command_button.bind(
            on_press=lambda btn: self.confirm_command_pressed(self.selection_id)
        )
        self.add_widget(send_command_button)

    def register_confirm_command_cb(self, func):
        if not callable(func):
            rospy.loginfo("Register confirm command cb is not callable")
        else:
            self.confirm_cb = func

    def command_selection_cb(self, spinner, selection):
        self.selection_id = selection

    def reset_selection(self):
        self.selection_id = "-"
        setattr(self.spinner, "text", "-")

    def parse_command_selection(self, text):
        if text == "-":
            return None
        # translate human readable command back to pin number
        # eg go from BOT_ON -> PIN_ON 7
        split = text.split("_")
        pin_type = split[0]
        pin_number = self.pin_name_dict[pin_type]
        pin_command = "PIN" + "_" + split[1] + " " + str(pin_number)
        return pin_command

    def confirm_command_pressed(self, text):
        command = self.parse_command_selection(text)
        if not command:
            print("Command {} was invalid".format(text))
            return
        rospy.loginfo("Confirming command: {} -> {}".format(command, text))
        if self.confirm_cb:
            self.confirm_cb(command)
        else:
            rospy.loginfo(
                "Error: Confirmation callback is None. Did you register callback with 'register_confirm_command_cb'"
            )
        self.reset_selection()


class PodControlPane(BoxLayout):
    motor_switch = ObjectProperty()
    pod_power_switch = ObjectProperty()
    spring_calibration_button = ObjectProperty()
    tether_speed_slider = ObjectProperty()
    tether_speed_text_input = ObjectProperty()
    pod_command_drop_down = PodCommandDropDown()

    def __init__(self, **kwargs):
        super(PodControlPane, self).__init__(**kwargs)

        self.pod_control = PodControl.get_instance(self.set_pod_calibration_results)

        self.motor_estop_sub = rospy.Subscriber(
            "/pod_motor_emergency_override", Bool, callback=self.emergency_stop_cb
        )
        self.tether_speed_sub = rospy.Subscriber(
            "/hullbot/pod/set_tether_speed", Float32, self.tether_speed_cb
        )
        # Upon starting, motors are off so spring calibration should not be possible
        self.spring_calibrated = False
        self.motors_on = False

        # Set the range of the tether speed slider
        self.tether_speed_slider.range = (PodControl.tether_speed_min, PodControl.tether_speed_max)

        # Register a cleanup function which will be executed at interpreter termination
        # Want to send 0 control speed commands to pod motors to ensure they stop
        atexit.register(self.emergency_stop_cb, Bool(True))

        self.pod_command_drop_down.register_confirm_command_cb(self.misc_pod_commands_callback)

    def emergency_stop_cb(self, msg):
        self.pod_control.emergency_stop(msg)

    def tether_speed_cb(self, msg):
        """
        Callback to update slider and text input when tether speed is set on the topic: /hullbot/pod/set_tether_speed
        Important to see if other modules update tether speed as well
        """
        raw_value = float(msg.data)

        # Round value to 2 decimal places for readibility
        value = round(raw_value, 2)

        # Update tether speed slider value and text input
        self.tether_speed_slider.value = value
        self.tether_speed_text_input.text = str(value)

    @threaded
    def calibrate_spring(self):
        # You should not be able to reel the tether while spring is calibrating
        self.tether_speed_slider.value = 0.0
        self.tether_speed_slider.disabled = True
        self.tether_speed_text_input.disabled = True

        self.pod_control.calibrate_spring()

    def set_pod_calibration_results(self, result):
        # You should be able to control the tether only when the spring has been calibrated
        if result:
            self.spring_calibrated = True
            self.tether_speed_slider.disabled = False
            self.tether_speed_text_input.disabled = False

    def send_pod_command(self, command):
        self.pod_control.send_pod_command(command)

    def pod_power_switch_callback(self, active, *args):
        self.pod_control.pod_power_switch_callback(active, args)

    def misc_pod_commands_callback(self, command):
        self.pod_control.misc_pod_commands_callback(command)

    def motor_switch_callback(self, active, *args):
        print(active)
        self.motors_on = self.pod_control.motor_switch_callback(active, args)

        # Spring calibration and the tether cannot work without the motors on
        if self.motors_on:
            self.spring_calibration_button.disabled = False

            # For the case where the motors were turned off and on after calibrating the spring
            if self.spring_calibrated:
                self.tether_speed_slider.disabled = False
        else:
            self.spring_calibration_button.disabled = True
            self.tether_speed_slider.disabled = True
            self.tether_speed_slider.value = 0.0

    def tether_slider_callback(self, raw_value, *args):
        if not self.pod_control.tether_slider_callback(raw_value, args):
            self.tether_speed_text_input.text = str(self.tether_speed_slider.value)
