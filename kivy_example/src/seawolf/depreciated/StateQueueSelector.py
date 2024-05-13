import json
import os
import sys

## Kivy imports are placed after standard library imports
import kivy
import rospy

# Ros imports
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolRequest, Trigger

kivy.require("1.2.0")
from ButtonsControl import ModeControl, RovioControl, SlamControl, StateQueueControl
from hullbot_msgs.srv import TrajectoryGeneratorSrv
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ListProperty, ObjectProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("stateQueueSelector.kv"))


class AutoModeIndicator(Label):
    statusColour = ListProperty([1, 0, 0, 0.5])  # set red by default

    def __init__(self, **kwargs):
        super(AutoModeIndicator, self).__init__(**kwargs)

    def alive(self):
        self.statusColour = [0.17, 0.3, 1, 0.5]  # set blue

    def dead(self):
        self.statusColour = [1, 1, 1, 0.4]  # set grey


class TofHoldIndicator(Label):
    statusColour = ListProperty([1, 0, 0, 0.5])  # set red by default

    def __init__(self, **kwargs):
        super(TofHoldIndicator, self).__init__(**kwargs)

    def alive(self):
        self.statusColour = [0.17, 0.3, 1, 0.5]  # set blue

    def dead(self):
        self.statusColour = [1, 1, 1, 0.4]  # set grey


class HorizonHoldIndicator(Label):
    statusColour = ListProperty([1, 0, 0, 0.5])  # set red by default

    def __init__(self, **kwargs):
        super(HorizonHoldIndicator, self).__init__(**kwargs)

    def alive(self):
        self.statusColour = [0.17, 0.3, 1, 0.5]  # set blue

    def dead(self):
        self.statusColour = [1, 1, 1, 0.4]  # set grey


class StateQueueSelector(BoxLayout):

    text_input = ObjectProperty(None)
    moon_land_button = ObjectProperty()
    mapping_mode_button = ObjectProperty()
    loaded_auto_script_abs = StringProperty()
    loaded_auto_script = StringProperty()
    current_state_name = StringProperty("")
    current_state_params = StringProperty("")
    current_state_param_vals = StringProperty("")
    region_selector_spinner = ObjectProperty()
    region_values = ListProperty()

    scheduler = 0

    def __init__(self, **kwargs):
        super(StateQueueSelector, self).__init__(**kwargs)

        # Primary controllers
        self.mode_control = ModeControl.get_instance()
        self.rovio_control = RovioControl.get_instance()
        self.queue_control = StateQueueControl.get_instance(self.set_state_queue_name)
        self.slam_control = SlamControl.get_instance(
            self.update_region_values_callback, self.dirty_regions_complete_callback
        )

        # Services
        self.find_dirty_regions_service = rospy.ServiceProxy(
            "/hullbot/octomap/find_fouling_regions_srv", Trigger
        )
        self.trajectory_generator_client = rospy.ServiceProxy(
            "/hullbot/trajectory_node/trajectory_generator_srv", TrajectoryGeneratorSrv
        )
        self.dynamic_trajectory_generator_client = rospy.ServiceProxy(
            "/hullbot/trajectory_node/dynamic_trajectory_generator_srv", TrajectoryGeneratorSrv
        )
        self.moon_land_client = rospy.ServiceProxy("/hullbot/moon_landing_node/moon_land", SetBool)

        # subscribe to necessary topics
        self.current_state_sub = rospy.Subscriber(
            "/hullbot/state_machine/current_state", String, self.current_state_cb
        )
        self.state_queue_sub = rospy.Subscriber(
            "/hullbot/state_machine/state_queue", String, self.state_queue_cb
        )
        self.enable_tof_hold_sub = rospy.Subscriber(
            "/hullbot/control_mode_handler/tof_hold_status", Bool, self.tof_hold_status_cb
        )
        self.horizon_hold_status_sub = rospy.Subscriber(
            "/hullbot/control_mode_handler/horizon_hold_status", Bool, self.horizon_hold_status_cb
        )
        self.auto_mode_flag_sub = rospy.Subscriber(
            "/hullbot/state_machine/auto_mode", Bool, self.auto_mode_flag_cb
        )

    def current_state_cb(self, msg):
        """
        Get current state of state machine
        """
        current_state = json.loads(msg.data)
        self.queue_control.current_state_str = msg.data
        self.current_state_name = current_state["state"]
        _current_state_params = _current_state_param_vals = ""
        for key, val in sorted(current_state["run_params"].items()):
            _current_state_params = _current_state_params + "\n" + key
            _current_state_param_vals = _current_state_param_vals + "\n" + str(val)
        self.current_state_params = _current_state_params
        self.current_state_param_vals = _current_state_param_vals

    def state_queue_cb(self, msg):
        """
        Get state queue from state machine
        """
        self.queue_control.state_queue = json.loads(msg.data)

        if (
            self.queue_control.state_queue_copy != self.queue_control.state_queue
        ):  # checking for the changes in the state queue list and updating the state queue display accordingly.
            self.queue_control.state_queue_copy = self.queue_control.state_queue
            # Delete Labels/Grids corresponding to the state that has just been completed
            for i in range(0, len(self.queue_control.grid_widgets)):
                self.ids.state_grid.remove_widget(
                    self.queue_control.grid_widgets[i]
                )  # remove labels from gridlayout widget
            self.queue_control.grid_widgets = (
                []
            )  # empty the label list, will be filled below with updated state queue
            for (
                state
            ) in (
                self.queue_control.state_queue
            ):  # for every state in the state queue, add a row of labels to the grid
                # Get state name, add label to grid row
                state_label = Label(text=state["state"])
                self.ids.state_grid.add_widget(state_label)
                self.queue_control.grid_widgets.append(state_label)

                # Get state params and vals
                state_params = state_param_vals = ""
                for key, val in sorted(state["params"].items()):
                    state_params = state_params + key + "\n"
                    state_param_vals = state_param_vals + str(val) + "\n"

                # Add params label to grid row
                param_label = Label(
                    text=state_params,
                    height=len(state_params) * 10,
                    halign="center",
                    valign="center",
                )
                self.ids.state_grid.add_widget(param_label)
                self.queue_control.grid_widgets.append(param_label)

                # Add params vals to grid row
                val_label = Label(
                    text=state_param_vals,
                    height=len(state_params) * 10,
                    halign="center",
                    valign="center",
                )
                self.ids.state_grid.add_widget(val_label)
                self.queue_control.grid_widgets.append(val_label)

    def auto_mode_flag_cb(self, msg):
        self.mode_control.auto_mode = msg.data
        if self.mode_control.auto_mode:
            self.ids.auto_mode_indicator.alive()
        else:
            self.ids.auto_mode_indicator.dead()

    def tof_hold_status_cb(self, msg):
        self.tof_hold_active = msg.data
        if self.tof_hold_active:
            self.ids.tof_hold_indicator.alive()
        else:
            self.ids.tof_hold_indicator.dead()

    def horizon_hold_status_cb(self, msg):
        self.horizon_hold_active = msg.data
        if self.horizon_hold_active:
            self.ids.horizon_hold_indicator.alive()
        else:
            self.ids.horizon_hold_indicator.dead()

    def moon_land(self):
        req = SetBoolRequest()
        req.data = True
        # Make the Moon landing button orangish meanwhile running
        self.moon_land_button.background_color = (1.0, 0.7, 0.0, 1.0)
        self.moon_land_button.text = "Moon land (working)"
        # Because moon land service takes around 20-30 seconds and freezes Seawolf, send it to a thread
        def call_moon_land(*args):
            """
            One-off anonymous function to do the service call in a thread
            """
            try:
                resp = self.moon_land_client(req)
                rospy.loginfo("Moon landing response: {}".format(resp))
                # Make the moon land button have a green background if successful, and red if not
                if resp.success:
                    self.moon_land_button.background_color = (0.0, 1.0, 0.0, 1.0)
                    self.moon_land_button.text = "Moon land (success)"
                else:
                    self.moon_land_button.background_color = (1.0, 0.0, 0.0, 1.0)
                    self.moon_land_button.text = "Moon land (failed)"
            except Exception as e:
                print("Exception on moonlanding: {}".format(e))
                rospy.logerr(e)
                self.moon_land_button.background_color = (1.0, 0.0, 0.0, 1.0)
                self.moon_land_button.text = "Moon land (exception)"

        t = rospy.Timer(rospy.Duration(0.0001), call_moon_land, oneshot=True)

    def start_rovio(self):
        self.rovio_control.start_rovio()

    def reset_octomap(self):
        self.slam_control.reset_octomap()

    def region_selector_pressed(self):
        self.slam_control.region_selector_pressed()

    def update_region_values_callback(self, values):
        self.region_values = values

    def find_dirty_regions(self):
        self.ids.dirty_region_button.text = "Getting Regions..."
        self.slam_control.find_dirty_regions()

    def dirty_regions_complete_callback(self):
        self.ids.dirty_region_button.text = "Find Dirty Regions"

    def clean_region(self):
        self.slam_control.clean_region(self.ids.region_selector_spinner.text)

    def enable_tof_hold(self):
        self.mode_control.enable_tof_hold()

    def disable_tof_hold(self):
        self.mode_control.disable_tof_hold()

    def enable_horizon_hold(self):
        self.mode_control.enable_horizon_hold()

    def disable_horizon_hold(self):
        self.mode_control.disable_horizon_hold()

    def toggle_auto_mode(self):
        self.mode_control.toggle_auto_mode()

    def clear_state_queue(self):
        self.queue_control.clear_state_queue()

    def load_states_to_queue(self, filename):
        self.queue_control.load_states_to_queue(filename)

    def set_state_queue_name(self, loaded_auto_script_abs):
        self.loaded_auto_script_abs = loaded_auto_script_abs
        self.loaded_auto_script = self.loaded_auto_script_abs.split("/")[-1]

    def show_load(self):
        self.queue_control.show_load()


if __name__ == "__main__":
    rospy.init_node("state_queue_selector")

    class WidgetApp(App):
        def build(self):
            return StateQueueSelector()

    WidgetApp().run()
