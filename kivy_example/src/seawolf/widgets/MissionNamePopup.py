#!/usr/bin/env python

import os
import string
from datetime import datetime

import kivy
import rospy

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from kivy.uix.popup import Popup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("missionNamePopup.kv"))

import Keypress


def get_date_YYMMDD():
    """
    Returns the date in the format YYMMDD_ as we use
    """
    now = datetime.now()
    year = now.strftime("%Y")[2:]
    date = year + now.strftime("%m%d")
    return date


def to_valid_chars(text):
    """
    Convert the string to valid characters for a filename,,
    substitutes any invalid character with underscore
    """
    valid_chars = "-_.%s%s" % (string.ascii_letters, string.digits)
    output_text = ""
    for c in text:
        if c in valid_chars:
            output_text += c
        else:
            output_text += "_"
    return output_text


class MissionNamePopup(Popup):
    """
    A popup which gets the user to provide a name for the rosbag,
    including the bot used, location and mission description
    """

    dropdown_bot = DropDown()
    dropdown_loc = DropDown()
    dropdown_boat = DropDown()

    def __init__(self, on_confirm_func, local=False, **kwargs):
        """
        on_confirm_func is called when the confirm button is pressed
        used to call start_rosbag_recording in RosbagOptionsPopup
        """

        super(MissionNamePopup, self).__init__(**kwargs)
        # Check explanation for the need of this at [1]
        # Create the date string looking like '200818_'
        self.current_date = get_date_YYMMDD()
        self.current_bot = ""
        self.current_location = ""
        self.current_boat_name = ""
        self.current_description = ""
        self.confirmation_input.text = self.current_date
        self.confirm_button.disabled = True
        self.is_local = local
        self.update_confirm_button_text()

        if callable(on_confirm_func):
            self.on_confirm_func = on_confirm_func
        else:
            raise TypeError("on_confirm_func is not callable")

        # We use the previousmission section to store the last mission
        # it gets recovered later when we start recording a rosbag
        config = App.get_running_app().config
        try:
            # TODO: The main section shouldnt be called the same?
            previous_mission_name = config.get("previousmission", "previousmissionname")
            if previous_mission_name == "":
                previous_mission_name = None
        except Exception as e:
            exception_str = "Got exception: {}.".format(e)
            exception_str += ", should mean that there is no previous mission name"
            print(exception_str)
            previous_mission_name = None

        # Deal with recovering the previous mission robot
        try:
            # TODO: The main section shouldnt be called the same?
            previous_mission_robot = config.get("previousmission", "previousmissionrobot")
            if previous_mission_robot == "":
                previous_mission_robot = None
        except Exception as e:
            exception_str = "Got exception: {}.".format(e)
            exception_str += ", should mean that there is no previous mission robot"
            print(exception_str)
            previous_mission_robot = None

        # Generate a dropdown with bot names as 'H6_XX'
        # Add options of the dropdown
        for bot_id in ["B", "BE", "C", "G", "H", "K", "N", "W", "X"]:
            btn = Button(text="H6_{}".format(bot_id), size_hint_y=None, height=80, width=200)
            btn.bind(on_release=lambda btn: self.dropdown_bot.select(btn.text))
            self.dropdown_bot.add_widget(btn)

        # Make the button have the bot name and not red anymore once chosen
        self.dropdown_bot.bind(
            on_select=lambda instance, x: setattr(self.bot_dropdown_button, "text", x)
        )
        # Default color is (1, 1, 1, 1)
        self.dropdown_bot.bind(
            on_select=lambda instance, x: setattr(
                self.bot_dropdown_button, "background_color", (1, 1, 1, 1)
            )
        )
        self.dropdown_bot.bind(on_select=self.update_confirmation_edit_by_bot)

        # Add function binding to dropdown selection
        self.dropdown_loc.bind(on_select=self.update_confirmation_edit_by_location)

        # Create a set of typical locations where we do missions
        typical_locations = sorted(["syd_bh", "ssm", "office", "darling_harbour"])

        # Add options of the dropdown
        for location in typical_locations:
            btn = Button(text=location, size_hint_y=None, height=80, width=200)
            btn.bind(on_release=lambda btn: self.dropdown_loc.select(btn.text))
            self.dropdown_loc.add_widget(btn)

        # Add function binding to dropdown selection
        self.dropdown_boat.bind(on_select=self.update_confirmation_edit_by_boat)

        # Create a set of typical boats where we do missions
        typical_boats = sorted(["ocean_adventurer", "ocean_tracker"])

        # Add options of the dropdown
        for boat in typical_boats:
            btn = Button(text=boat, size_hint_y=None, height=80, width=200)
            btn.bind(on_release=lambda btn: self.dropdown_boat.select(btn.text))
            self.dropdown_boat.add_widget(btn)

        # Initial label for the location dropdown button
        self.location_dropdown_button.text = "Common"

        # Initial label for the boat dropdown button
        self.boat_dropdown_button.text = "Common"

        # TODO: Add chosing a folder would be nice but looks like a lot of work

    def on_open(self):
        """
        Deactivate keyboard control while choosing a new rosbag name
        """
        Keypress.KEY_ACTIVE = False

    def on_dismiss(self):
        """
        Reactivate keyboard control once it the popup is dismissed
        """
        Keypress.KEY_ACTIVE = True

    def update_confirmation_edit_by_bot(self, kivy_object, dropdown_value):
        """
        Callback to be called when the dropdown of the bot has been selected that updates
        the confirmation input text box.
        """
        self.current_bot = dropdown_value
        self.enable_confirm_if_fields_populated()

    def update_confirmation_edit_by_location(self, kivy_object, dropdown_value):
        """
        Callback to be called when the dropdown of the location has been selected that updates
        the confirmation input text box.
        """

        # Change the text field to the selected dropdown value
        self.location_on_text(dropdown_value)

    def location_on_text(self, text):
        """
        Callback for when the location text is edited
        """

        # Make text valid
        text = to_valid_chars(text)

        # Update the text box for user feedback
        self.location_text.text = text

        # Update the current_location variable
        self.current_location = text

        # Update the final rosbag name
        self.update_confirmation_text()
        self.enable_confirm_if_fields_populated()

    def update_confirmation_edit_by_boat(self, kivy_object, dropdown_value):
        """
        Callback to be called when the dropdown of the boat has been selected that updates
        the confirmation input text box.
        """

        # Change the text field to the selected dropdown value
        self.boat_on_text(dropdown_value)

    def boat_on_text(self, text):
        """
        Callback for when the boat text is edited
        """

        # Make text valid
        text = to_valid_chars(text)

        # Update the text box for user feedback
        self.boat_name_text_input.text = text

        # Update the boat variable
        self.current_boat_name = text

        # Update the final rosbag name
        self.update_confirmation_text()
        self.enable_confirm_if_fields_populated()

    def confirm_mission_naming(self, mission_name):
        """
        This callback is called when the confirm button is pressed.
        """
        # Store the full mission name for the folder that will contain the rosbags
        previous_mission_name = self.confirmation_input.text
        try:
            App.get_running_app().config.set(
                "previousmission", "previousmissionname", previous_mission_name
            )
            App.get_running_app().config.write()
        except Exception as e:
            print("Exception on writing config: {}".format(e))

        # Store bot (used on rosbag name)
        previous_mission_robot = self.current_bot
        try:
            App.get_running_app().config.set(
                "previousmission", "previousmissionrobot", previous_mission_robot
            )
            App.get_running_app().config.write()
        except Exception as e:
            print("Exception on writing config: {}".format(e))

        # Set rosparam to track current boat.  This is used for saving/loading slam atlases
        rospy.set_param("current_boat", self.current_boat_name)
        # Store mission description (used on rosbag name)
        previous_mission_description = (
            self.current_boat_name + "_" + self.current_location + "_" + self.current_description
        )
        try:
            App.get_running_app().config.set(
                "previousmission", "previousmissiondescription", previous_mission_description
            )
            App.get_running_app().config.write()
        except Exception as e:
            print("Exception on writing config: {}".format(e))

        self.on_confirm_func()
        self.dismiss()

    def bypass_rosbag_record(self):
        """
        Uncomment the button in missionNamePopup.kv in order to allow
        skipping of rosbag recording for development purposes
        """
        try:
            self.dismiss()
        except Exception as e:
            print("Error bypassing rosbag record: ", e)

    def description_on_text(self, text):
        """
        Parse the text entered and make sure it's file-name friendly.
        """
        text = to_valid_chars(text)
        self.description_text_input.text = text
        self.current_description = text

        # Update the final rosbag name
        self.update_confirmation_text()
        self.enable_confirm_if_fields_populated()

    def boat_name_on_text(self, text):
        """
        Parse the text entered and make sure it's file-name friendly.
        """
        self.current_boat_name = to_valid_chars(text)
        self.boat_name_text_input.text = self.current_boat_name
        rospy.set_param("boat_name", self.current_boat_name)

        # Update the final rosbag name
        self.update_confirmation_text()
        self.enable_confirm_if_fields_populated()

    def confirmation_on_text(self, text):
        """
        Parse the text entered and make sure it's file-name friendly.
        """
        text = to_valid_chars(text)
        self.confirmation_input.text = text

    def update_confirmation_text(self):
        self.confirmation_input.text = self.current_date + "_" + self.current_boat_name

    def enable_confirm_if_fields_populated(self):
        """
        Check if all required fields are populated, enable the button if so, otherwise, make sure it's disabled.
        """
        if (
            self.current_bot != ""
            and self.current_location != ""
            and self.current_description != ""
            and self.current_boat_name != ""
        ):
            self.confirm_button.disabled = False
        else:
            self.confirm_button.disabled = True

    def update_confirm_button_text(self):
        """ """
        if self.is_local:
            self.confirm_button.text = "Confirm Mission Naming (recording locally)"
        else:
            self.confirm_button.text = "Confirm Mission Naming (recording on topside)"

    def change_recording_location(self, local):
        """
        Change between local and topside recording
        """
        self.is_local = local
        self.update_confirm_button_text()


if __name__ == "__main__":

    class WidgetApp(App):
        def build(self):
            return MissionNamePopUp()

    WidgetApp().run()
