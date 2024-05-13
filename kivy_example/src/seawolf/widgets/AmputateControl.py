import re

import kivy
import rospy

kivy.require("1.2.0")
from hullbot_msgs.srv import SetBoolArray, SetBoolArrayRequest, SetBoolArrayResponse
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("amputateControl.kv"))


class AmputateControl(BoxLayout):
    """This widget enables amputation of a single thruster for motion controller output.

    This means it is possible to swim the bot with 7 motors with slightly worse performance instead
    of requiring all 8 motors. This is helpful as an emergency measureif a motor breaks outs on operations.
    This simply sends a service request and no feedback on the thruster enable mask is avaliable.
    """

    def __init__(self, **kwargs):
        super(AmputateControl, self).__init__(**kwargs)
        self._tc = None
        avaliable_tcs = ["None", "TC1", "TC2", "TC3", "TC4", "TC5", "TC6", "TC7", "TC8"]
        self._thruster_enable_mask_client = rospy.ServiceProxy(
            "~thruster_enable_mask", SetBoolArray
        )
        self._create_dropdown("tc_selector", avaliable_tcs, self._update_thruster_enable_mask)
        self.ids.tc_selector.set_false()

    def _create_dropdown(self, button_id, options, button_func):
        """Creates a dropdown for a button given the button id and options.

        Args:
            button_id (string): Button of id defined in kv file.
            options (list): Possible options for dropdown.
            button_func (func): Function callback for button.
        """
        dropdown = DropDown()
        for option in options:

            option_str = str(option)

            btn = Button(
                text=option_str,
                size_hint_y=None,
                height=30,
            )
            btn.bind(on_release=lambda btn: dropdown.select(btn.text))
            dropdown.add_widget(btn)

        # Dynamically bind the button with function to open dropdown
        button_bind_func = getattr(self.ids[button_id], "bind")
        button_bind_func(on_release=dropdown.open)

        # Dynamically bind the dropdown with function to save option
        dropdown.bind(on_select=button_func)

        setattr(self, button_id + "_dropdown", dropdown)

    def _update_thruster_enable_mask(self, obj, tc):
        """Update the thruster enable mask by constructing bool data and sending service call.

        Args:
            obj (Kivy object): Kivy object passed when binding. If manually calling, pass None.
            tc (string): Selected TC.
        """

        # Save tc selection and update text
        self._tc = tc
        setattr(self.ids["tc_selector"], "text", self._tc)

        # Extract selected TC from text
        tc_number = self._extract_number(self._tc)

        # Create bool array data
        bool_array = [True for _ in range(8)]

        # If valid TC selected, set Bool array data to false
        if tc_number:
            idx = tc_number - 1
            bool_array[idx] = False
            self.ids.tc_selector.set_true()
        else:
            self.ids.tc_selector.set_false()

        # Make service call
        response = self._set_thruster_enable_mask(bool_array)
        if not response.success:
            rospy.logerr(response.message)

    def _extract_number(self, tc_string):
        """Return INT if found in string format TC*.
        If not found, return None.

        Args:
            tc_string (string): String

        Returns:
            int: Integer
        """
        pattern = r"TC(\d+)"
        match = re.search(pattern, tc_string)

        if match:
            extracted_number = match.group(1)
            return int(extracted_number)
        else:
            return None

    def _set_thruster_enable_mask(self, data):
        """Set thruster enable mask via service call.

        Args:
            data (bool[]): Bool Array

        Returns:
            SetBoolArrayResponse: Response.
        """
        try:
            return self._thruster_enable_mask_client(SetBoolArrayRequest(data=data))
        except rospy.ServiceException as err:
            message = "Thruster enable mask service call failed: " + str(err)
            return SetBoolArrayResponse(False, message)
