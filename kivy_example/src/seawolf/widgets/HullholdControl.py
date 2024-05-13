import kivy
import rospy

kivy.require("1.2.0")
from hullbot_msgs.srv import SetFloat, SetFloatRequest, SetFloatResponse
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("hullholdControl.kv"))


class HullholdControl(BoxLayout):
    """This widget controls the hullhold functionality, currently only the steady state thrust magnitude.

    1. Update the steady state thrust magnitude when corresponding button i.e Light, Medium or Heavy is pressed.
        - Sends a service call to mission manager, which sends a service call to motion controller.

    2. Indicate to the user the current steady state thrust magnitude.
        - Updates the button i.e Light, Medium or Heavy to green if steady state thrust magnitude matches the corresponding level.
        - Updates all buttons to grey if steady state thrust magnitude cannot be fetched or does not match determined levels.
        - Steady state thrust magnitude is fetched via parameters.
        - Steady state thrust parameter feedback eliminates information disparity if seawolf or motion controller restarts during operation.
    """

    STEADY_STATE_THRUST_LEVELS = {"light": 1.25, "medium": 1.5, "heavy": 1.75}

    def __init__(self, **kwargs):
        super(HullholdControl, self).__init__(**kwargs)
        self._mission_steady_state_thrust_client = rospy.ServiceProxy(
            "~mission_steady_state_thrust", SetFloat
        )

        update_rate = 0.2  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))

    def _update_graphics_cb(self, *args):
        """Update the button colors based on the steady state thrust magnitude.

        If magnitude found in steady state thrust mapping, set corresponding button to green.
        If magnitude not found in steady state thrust mapping, set all buttons to grey.
        """

        # Set all buttons to grey
        self._set_button_neutral("light")
        self._set_button_neutral("medium")
        self._set_button_neutral("heavy")

        # Check if desired parameter exists
        if rospy.has_param("~hullhold_steady_state_thrust"):

            # Wrap this in a try-except as there is a non-zero chance
            # the parameter could be cleared after checking it exists and before getting the parameter
            # as motion controller clears its node namespace parameters before launching
            # resulting in a key error, which crashes seawolf
            try:
                # Get value from desired parameter
                value = float(rospy.get_param("~hullhold_steady_state_thrust"))

                # Get corresponding key of value
                key = self._find_key_by_value(self.STEADY_STATE_THRUST_LEVELS, value)

                # If key is valid, set button color to green
                if key:
                    key = key.lower()
                    self._set_button_true(key)
            except KeyError as e:
                pass

    def _find_key_by_value(self, dictionary, value_to_find):
        """Find key of value in a dictionary.

        Args:
            dictionary (dict): Dictionary to search.
            value_to_find (float): Value to find.

        Returns:
            _type_: _description_
        """
        for key, value in dictionary.items():
            if value == value_to_find:
                return key
        return None

    def update_steady_state_thrust_based_on_level(self, level_str):
        """Update steady state thrust to magnitude corresponding based on desired magnitude.

        If service call is successful, immediately update graphics.

        Args:
            level_str (string): String indicating level
        """

        level = str(level_str).lower()
        value = self.STEADY_STATE_THRUST_LEVELS[level]
        response = self._set_steady_state_thrust(value)
        rospy.loginfo(response)
        if response.success:
            self._update_graphics_cb()

    def _set_steady_state_thrust(self, req):
        """Set hullhold steady state thrust via service call.

        Args:
            req (float): Steady state thrust value

        Returns:
            SetFloatResponse: Response
        """
        try:
            return self._mission_steady_state_thrust_client(SetFloatRequest(data=req))
        except rospy.ServiceException as err:
            message = "Hullhold steady state thrust service call failed: " + str(err)
            return SetFloatResponse(False, message)

    def _set_button_true(self, level):
        """Set button to True i.e Green.

        Args:
            level (string): String indicating level.
        """
        button_name = "{}_button".format(level)
        if hasattr(self.ids, button_name):
            button = getattr(self.ids, button_name)
            button.set_true()

    def _set_button_neutral(self, level):
        """Set button to Neutral i.e Grey.

        Args:
            level (string): String indicating level.
        """
        button_name = "{}_button".format(level)
        if hasattr(self.ids, button_name):
            button = getattr(self.ids, button_name)
            button.set_neutral()
