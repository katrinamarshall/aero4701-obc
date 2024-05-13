import atexit
import os
from tokenize import String

import kivy
import rospy
from std_msgs.msg import Bool, Float32

from seawolf.utils.customWidgets import PositiveTextInput
from seawolf.utils.ThreadingSupport import threaded

kivy.require("1.2.0")
from AmputateControl import AmputateControl
from ButtonsControl import (
    ButtonsControl,
    CleaningControl,
    LightControl,
    ProcessControl,
    TurboControl,
    DepthLockControl,
)
from HullholdControl import HullholdControl
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ListProperty, StringProperty
from kivy.uix.tabbedpanel import TabbedPanel
from SystemTest import SystemTest
from TrajectoryControl import TrajectoryControl

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("controlPane.kv"))


class ControlPane(TabbedPanel):
    """
    Demo view buttons pane that provides many base functionalities (startup, shutdown, rovio, modes, rosbag, etc.)
    """

    def __init__(self, **kwargs):
        super(ControlPane, self).__init__(**kwargs)

        # Controller instances to provide shared functionality
        self.buttons_controller = ButtonsControl.get_instance()
        self.light_control = LightControl.get_instance()
        self.process_control = ProcessControl.get_instance()
        self.cleaning_control = CleaningControl.get_instance()
        self.turbo_control = TurboControl.get_instance()

        self.depth_lock_control = DepthLockControl.get_instance(self.ids.depth_lock_setter)
        self.ids.trajectory_control.add_widget(TrajectoryControl())
        self.system_test = SystemTest()
        self.ids.amputate_control.add_widget(AmputateControl())
        self.ids.hullhold_control.add_widget(HullholdControl())

        # Initialise button indicator colours
        self.ids.top_lights_button.set_false()
        self.ids.glow_lights_button.set_false()
        self.ids.front_lights_button.set_false()
        self.ids.side_lights_button.set_false()

        self.ids.start_inspection_button.set_true()
        self.ids.stop_inspection_button.set_false()
        self.ids.enable_joust_button.set_true()
        self.ids.disable_joust_button.set_false()

        self.ids.enable_turbo_mode_button.set_true()
        self.ids.disable_turbo_mode_button.set_false()

        # Register button color functions so debug control has access for button indication.
        self.process_control.register_reset_neutral(self.ids.reset_motors_button.set_black)
        self.process_control.register_reset_true(self.ids.reset_motors_button.set_true)
        self.process_control.register_reset_warn(self.ids.reset_motors_button.set_warn)

        self.light_buttons = {
            "top": self.ids.top_lights_button,
            "glow": self.ids.glow_lights_button,
            "front": self.ids.front_lights_button,
            "side": self.ids.side_lights_button,
        }

        self.light_sliders = {
            "top": self.ids.top_lights_intensity_slider,
            "glow": self.ids.glow_lights_intensity_slider,
            "front": self.ids.front_lights_intensity_slider,
            "side": self.ids.side_lights_intensity_slider,
        }
        self.LIGHT_DIM_VALUE = 5
        self.ids.selected_light_groups_button.set_label(
            self.light_control.get_selected_light_type()
        )

    def on_startup(self):
        self.buttons_controller.start_and_launch_topside()
        self.buttons_controller.launch_botside()

    def on_shutdown(self):
        self.buttons_controller.kill_topside()
        self.buttons_controller.kill_botside()

    def launch_topside(self):
        self.buttons_controller.start_and_launch_topside()

    def launch_botside(self):
        self.buttons_controller.launch_botside()

    def kill_topside(self):
        self.buttons_controller.kill_topside()

    def kill_botside(self):
        self.buttons_controller.kill_botside()

    def on_launch_rviz(self):
        self.buttons_controller.launch_rviz()

    def on_launch_robot_monitor(self):
        self.buttons_controller.launch_robot_monitor()

    def on_launch_plotjuggler(self):
        self.buttons_controller.launch_plotjuggler()

    def on_launch_smach_viewer(self):
        self.buttons_controller.launch_smach_viewer()

    def on_rosbag_options(self):
        self.buttons_controller.open_rosbag_popup()

    def toggle_light_group_state(self, light_group_str):
        """Toggle light group state.

        Args:
            light_group_str (str): light group name i.e top, glow etc. (Must be valid LightGroup object name).
        """
        light_group = str(light_group_str).lower()
        state = not self.light_control.get_light_group_state(light_group)
        self.set_light_group_state(light_group, state)

    def set_light_group_state(self, light_group_str, state):
        """Set light group state and set button state to correspond with light group state.

        Args:
            light_group_str (str): light group name i.e top, glow etc. (Must be valid LightGroup object name).
        """
        light_group = str(light_group_str).lower()
        self.light_control.set_light_group_state(light_group, state)
        self.light_buttons[light_group].set_state(
            self.light_control.get_light_group_state(light_group)
        )

    def set_light_group_brightness(self, light_group_str, val):
        """Set light group brightness after remapping range.

        Args:
            light_group_str (str): light group name i.e top, glow etc. (Must be valid LightGroup object name).
            val (int): brightness value [0 - 255].
        """
        light_group = str(light_group_str).lower()
        remapped_val = self.remap_range(val, 0, 100, 0, 255)
        self.light_control.set_light_group_brightness(light_group, remapped_val)

    def remap_range(self, x, in_min, in_max, out_min, out_max):
        """Re-maps a number from one range to another."""

        if in_min >= in_max:
            raise ValueError("in_min must be less than in_max")
        if out_min >= out_max:
            raise ValueError("out_min must be less than out_max")
        if not in_min <= x <= in_max:
            raise ValueError("x must be between in_min and in_max")
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def set_light_preset_to_on(self):
        """Set lights to ON by setting all light groups to true."""
        for light_group in self.light_sliders:
            self.set_light_group_state(light_group, True)

    def set_light_preset_to_dim(self):
        """Set lights to DIM by setting light sliders and setting all light groups to true."""
        for light_group in self.light_sliders:
            self.set_light_group_state(light_group, True)

        for light_group in self.light_sliders:
            self.light_sliders[light_group].value = self.LIGHT_DIM_VALUE

    def set_light_preset_to_off(self):
        """Set lights to OFF by setting all light groups to false."""
        for light_group in self.light_sliders:
            self.set_light_group_state(light_group, False)

    def toggle_selected_light_type(self):
        """Set lights to a new light type i.e different physical light configuration."""
        new_light_type = self.light_control.toggle_selected_light_type()
        self.ids.selected_light_groups_button.set_label(new_light_type)

        # Set all buttons and sliders back to OFF
        for light_group in self.light_sliders:
            self.light_sliders[light_group].value = 0
            self.set_light_group_state(light_group, False)

    def reset_tmescs(self):
        self.process_control.reset_tmescs()

    def start_inspection_camera(self):
        self.process_control.start_inspection_camera()

    def stop_inspection_camera(self):
        self.process_control.stop_inspection_camera()

    def enable_joust(self):
        self.cleaning_control.enable_joust()

    def disable_joust(self):
        self.cleaning_control.disable_joust()

    def fix_socketcan_crash(self):
        self.process_control.fix_socketcan_crash()

    def fix_network_dropout(self):
        self.process_control.fix_network_dropout()

    def fix_imu_dropout(self):
        self.process_control.fix_imu_dropout()

    def enable_turbo_mode(self):
        self.turbo_control.enable_turbo_mode()

    def disable_turbo_mode(self):
        self.turbo_control.disable_turbo_mode()


if __name__ == "__main__":

    class WidgetApp(App):
        def build(self):
            return ControlPane()

    rospy.init_node("control_pane_test")
    WidgetApp().run()
