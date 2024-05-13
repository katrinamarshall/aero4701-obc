import kivy
import rosnode
import rospy
from rospy.exceptions import ROSException

from seawolf.utils.ThreadingSupport import threaded

kivy.require("1.2.0")
from hb_navigation_msgs.srv import PlanTrajectoryRequest
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from kivy.uix.relativelayout import RelativeLayout
from std_msgs.msg import String

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("trajectoryControl.kv"))


class TrajectoryControl(RelativeLayout):
    """
    This widget alters the requested trajectory parameters in mission_manager
    for dynamic changing of trajectories during runtime.
    """

    def __init__(self, **kwargs):
        super(TrajectoryControl, self).__init__(**kwargs)
        self._trajectory_node = "/trajectory"
        self._mission_manager_node = "/mission/mission_manager"
        self._speed = None
        self._path = None
        self._orientation = None
        self._alignment = None
        self._interpolation = None
        self.ids.update_trajectory.set_true()
        self.wait_and_fetch_mission_trajectory_parameters()

        self._transition_pub = rospy.Publisher("~state_transitions", String, queue_size=1)
        self.ids.enter_exit_trajectory_freeswim.set_true()

        self.current_state = None
        self._state_sub = rospy.Subscriber("~state", String, callback=self._state_cb)

    def _state_cb(self, msg):
        """Store state and set button accordingly
        Args:
            msg (String): msg
        """

        # If trajectory has finished, reset button
        if self.current_state == "Following Trajectory" and msg.data == "Finished Trajectory":
            self.ids.enter_exit_trajectory_freeswim.text = "Enter Trajectory Freeswim"
            self.ids.enter_exit_trajectory_freeswim.set_true()

        self.current_state = msg.data

    def _create_all_dropdowns(self):
        """Create dropdowns for the trajectory parameter options.

        This is called once the avaliable options are fetched.
        For the avaliable paths, it fetches the paths from the trajectory node parameters.
        Therefore, this is called only once that node is up.
        """
        avaliable_paths = self._get_avaliable_paths()
        avaliable_speeds = [0.02, 0.05, 0.075, 0.1, 0.125, 0.15]
        avaliable_orientations = [True, False]
        avaliable_alignment = {PlanTrajectoryRequest.NONE, PlanTrajectoryRequest.HORIZON}
        avaliable_interpolations = {
            PlanTrajectoryRequest.LINEAR,
            PlanTrajectoryRequest.LIPB,
            PlanTrajectoryRequest.CUBIC,
            PlanTrajectoryRequest.DUBINS,
        }
        self._create_dropdown("path_selector", avaliable_paths, self._store_path_parameter)
        self._create_dropdown("speed_selector", avaliable_speeds, self._store_speed_parameter)
        self._create_dropdown(
            "orientation_selector", avaliable_orientations, self._store_orientation_parameter
        )
        self._create_dropdown(
            "alignment_selector", avaliable_alignment, self._store_alignment_parameter
        )
        self._create_dropdown(
            "interpolation_selector", avaliable_interpolations, self._store_interpolation_parameter
        )

    def _create_dropdown(self, button_id, options, update_config_func):
        """Creates a dropdown for a button given the button id and options.

        Args:
            button_id (string): Button of id defined in kv file.
            options (list): Possible options for dropdown.
            update_config_func (func): Function to save selected option.
        """
        dropdown = DropDown()
        for option in options:

            option_str = str(option)

            btn = Button(
                text=option_str,
                size_hint_y=None,
                height=40,
            )
            btn.bind(on_release=lambda btn: dropdown.select(btn.text))
            dropdown.add_widget(btn)

        # Dynamically bind the button with function to open dropdown
        button_bind_func = getattr(self.ids[button_id], "bind")
        button_bind_func(on_release=dropdown.open)

        # Dynamically bind the dropdown with function to save option
        dropdown.bind(on_select=update_config_func)

        setattr(self, button_id + "_dropdown", dropdown)

    def _store_path_parameter(self, obj, path):
        """Store selected path parameter and update button text

        Args:
            obj (Kivy object): Kivy object passed when binding. If manually calling, pass None.
            path (string): Selected path.
        """
        self._path = path
        setattr(self.ids["path_selector"], "text", self._path)

    def _store_speed_parameter(self, obj, speed):
        """Store selected speed parameter and update button text

        Args:
            obj (Kivy object): Kivy object passed when binding. If manually calling, pass None.
            speed (string): Selected speed.
        """
        # Convert string number to float
        self._speed = float(speed)
        setattr(self.ids["speed_selector"], "text", str(self._speed))

    def _store_orientation_parameter(self, obj, orientation):
        """Store selected orientation parameter and update button text

        Args:
            obj (Kivy object): Kivy object passed when binding. If manually calling, pass None.
            orientation (string): Selected orientation.
        """
        # Convert string "True" or False to boolean
        try:
            if type(orientation) != bool:
                self._orientation = eval(orientation)
        except NameError as e:
            rospy.loginfo(
                "Not a valid orientation mode. Must be True or False boolean. {}".format(e)
            )

        setattr(self.ids["orientation_selector"], "text", str(self._orientation))

    def _store_alignment_parameter(self, obj, alignment):
        """Store selected alignment parameter and update button text

        Args:
            obj (Kivy object): Kivy object passed when binding. If manually calling, pass None.
            alignment (string): Selected alignment.
        """
        self._alignment = alignment
        self.ids["alignment_selector"].text = self._alignment

    def _store_interpolation_parameter(self, obj, interpolation):
        """Store selected interpolation parameter and update button text

        Args:
            obj (Kivy object): Kivy object passed when binding. If manually calling, pass None.
            interpolation (string): Selected interpolation.
        """
        self._interpolation = interpolation
        setattr(self.ids["interpolation_selector"], "text", self._interpolation)

    def _get_avaliable_paths(self):
        """Get avaliable path types from trajectory node by parsing the parameters.

        i.e rectangle/default
            rectangle/preset_1
            rectangle/preset_2

        Returns:
            string[]: Avaliable path type options.
        """
        path_dict = rospy.get_param("/trajectory/path/")
        key_combinations = []
        for key, value in path_dict.iteritems():
            if isinstance(value, dict):
                nested_keys = [key + "/" + nested_key for nested_key in value.keys()]
                key_combinations.extend(nested_keys)
        return key_combinations

    @threaded
    def wait_and_fetch_mission_trajectory_parameters(self):
        """Waits until requirements are met before enabling the control interface.

        Waits for the mission manager and trajectory pipeline node to be up.
        before fetching the current trajectory parameters and populating the dropdown options.
        Therefore, the control interface is not intialised or usable until conditions are met.
        """
        while self._empty_stored_parameters():

            if self._mission_manager_up() and self._trajectory_pipeline_up():

                self._get_mission_trajectory_parameters()

                self._store_path_parameter(None, self._path)
                self._store_speed_parameter(None, self._speed)
                self._store_orientation_parameter(None, self._orientation)
                self._store_alignment_parameter(None, self._alignment)
                self._store_interpolation_parameter(None, self._interpolation)

                self._create_all_dropdowns()
                rospy.loginfo("Succesfully fetched parameters and initialised interface.")

            rospy.sleep(3.0)

    def update_mission_trajectory_parameters(self):
        """Update the requested trajectory parameters in the mission manager.

        Successful only if stored parameters are not None and mission manager is up.
        """
        if self._mission_manager_up():
            if not self._empty_stored_parameters():
                self._set_mission_trajectory_parameters()
                rospy.loginfo("Successfully updated trajectory parameters in mission_manager!")
            else:
                rospy.loginfo(
                    "Failed to update trajectory parameters as not all parameters are filled."
                )
        else:
            rospy.loginfo("Failed to update trajectory parameters as mission manager is not up.")

    def _set_mission_trajectory_parameters(self):
        """Set mission trajectory parameters."""
        rospy.set_param("~mission_trajectory_path", self._path)
        rospy.set_param("~mission_trajectory_speed", self._speed)
        rospy.set_param("~mission_trajectory_orientation", self._orientation)
        rospy.set_param("~mission_trajectory_alignment", self._alignment)
        rospy.set_param("~mission_trajectory_interpolation", self._interpolation)

    def _get_mission_trajectory_parameters(self):
        """Get mission trajectory parameters."""
        self._path = rospy.get_param("~mission_trajectory_path")
        self._speed = rospy.get_param("~mission_trajectory_speed")
        self._orientation = rospy.get_param("~mission_trajectory_orientation")
        self._alignment = rospy.get_param("~mission_trajectory_alignment")
        self._interpolation = rospy.get_param("~mission_trajectory_interpolation")

    def _empty_stored_parameters(self):
        """Check if stored parameters are empty i.e None."""
        return (
            self._path == None
            or self._speed == None
            or self._orientation == None
            or self._alignment == None
        )

    def _mission_manager_up(self):
        """Check if mission manager node is up.

        Returns:
            bool: Node status
        """
        return self._node_up(self._mission_manager_node)

    def _trajectory_pipeline_up(self):
        """Check if trajectory pipeline node is up.

        Returns:
            bool: Node status
        """
        return self._node_up(self._trajectory_node)

    def _node_up(self, node):
        """Check if a node is up.

        Args:
            node (string): Node name

        Returns:
            bool: Node status
        """
        return node in rosnode.get_node_names()

    @threaded
    def enter_exit_trajectory_freeswim(self):

        # Publish the trigger to transition states
        self._publish_transition("trajectory_freeswim_trigger")

        # Wait for state transition message
        try:
            rospy.sleep(0.2)
            msg = rospy.wait_for_message("~state", String, 3)
            state = msg.data

            # Update the button graphics only if state is updated
            if (
                state == "Requesting Trajectory"
                and self.ids.enter_exit_trajectory_freeswim.text == "Enter Trajectory Freeswim"
            ):
                self.ids.enter_exit_trajectory_freeswim.text = "Exit Trajectory Freeswim"
                self.ids.enter_exit_trajectory_freeswim.set_false()
            elif (
                state != "Requesting Trajectory"
                and self.ids.enter_exit_trajectory_freeswim.text == "Exit Trajectory Freeswim"
            ):
                self.ids.enter_exit_trajectory_freeswim.text = "Enter Trajectory Freeswim"
                self.ids.enter_exit_trajectory_freeswim.set_true()
        except ROSException as e:
            pass

    def _publish_transition(self, data):
        msg = String(data=data)
        self._transition_pub.publish(msg)
