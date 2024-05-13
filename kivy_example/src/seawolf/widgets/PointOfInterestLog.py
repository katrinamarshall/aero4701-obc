import os
import time

import kivy

kivy.require("1.2.0")

import Keypress
import rospy
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ObjectProperty
from kivy.uix.popup import Popup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("pointOfInterestLog.kv"))


class PointOfInterestLog(Popup):

    text_input = ObjectProperty(None)

    def __init__(self, **kwargs):
        super(PointOfInterestLog, self).__init__(**kwargs)

        self.log_file_name = "Default_log_file"
        self.log_contents = ""
        self.bag_name = ""
        self.bag_loc = ""
        self.last_entry_prefix = ""
        self.kp = Keypress.KeyHandler()
        # This is for the keybind for closing the POI window, bound to 'ENTER'
        self.kp.bind_key_to_func("\r", self.on_save_button, "POI", "close POI")

    def on_open(self):
        """
        Deactivate keyboard control, load in log file, append new timestamp
        """
        self.log_contents = self.load_log(self.log_file_name)
        modified_log = self.append_new_log_entry(self.log_contents)
        self.load_log_into_text_box(modified_log)
        Keypress.KEY_ACTIVE = False
        Keypress.POI_ACTIVE = True

    def on_dismiss(self):
        """
        Reactivate keyboard control once it the popup is dismissed
        """
        Keypress.KEY_ACTIVE = True
        Keypress.POI_ACTIVE = False
        self.clear_textbox()

    def clear_textbox(self):
        self.text_input.text = ""

    def load_log_into_text_box(self, text):
        self.text_input.text = text

    def load_log(self, log_file):
        """Loads in the contents of log if the file exists
        Args:
            log_file ([str]): Location of file to load

        Returns:
            [str]: Contents of the log file
        """
        log_contents = ""
        if self._log_exists(log_file):
            with open(log_file, "r") as stream:
                log_contents = stream.read()
        else:
            rospy.loginfo("Could not load log file")

        return log_contents

    def append_new_log_entry(self, text):
        """Appends a new timestamp to the log contents

        Args:
            text ([str]): Contents of the log file

        Returns:
            [str]: Returns the log contents with new timestamp appended
        """
        t = time.localtime()
        current_time = time.strftime("%H:%M:%S", t)
        ros_time = rospy.Time.now().to_sec()
        new_text = "{}  ({}): ".format(current_time, ros_time)
        self.last_entry_prefix = new_text
        return "{}\n{}".format(text, new_text)

    def on_save_button(self):
        """Saves the log and publishes the last change made to the logs"""
        text_input = self.text_input.text
        last_entry = self.find_latest_log_entry(self.last_entry_prefix, text_input)
        self.save(text_input, self.log_file_name)
        self.publish_log_entry(last_entry)
        self.dismiss()  # closes popup once done

    def find_latest_log_entry(self, last_entry_prefix, text):
        """Locates and returns the text following the prefix provided

        Args:
            last_entry_prefix (str): The timestamp that was appended before last entry
            text (str): The contents of the log file
        Returns:
            [str]: The last log file entry
        """
        index = text.find(last_entry_prefix)
        shifted_index = index + len(last_entry_prefix)
        return text[shifted_index:]

    def publish_log_entry(self, text):
        rospy.loginfo("Point of interest: {}".format(text))

    def save(self, text, file_name):
        """
        writes text from widget editing window to selected file
        """
        if text != "":
            with open(file_name, "w") as stream:
                stream.write(text)

    def update_bag_name(self, curr_bag, mission_name):
        self.bag_name = curr_bag
        self.bag_loc = os.path.expanduser("~/Hullbot_bag_files/") + mission_name + "/"
        if not os.path.exists(self.bag_loc):
            try:
                os.makedirs(self.bag_loc)
            except Exception as e:
                err = "Exception when trying to create folder ({}) to save POI: {}".format(
                    self.bag_loc, e
                )
                print(err)
                rospy.logerr(err)

        self.log_file_name = "{}{}POI.txt".format(self.bag_loc, self.bag_name)
        print("Log file: {}".format(self.log_file_name))

    @staticmethod
    def _log_exists(log_contents):
        return os.path.isfile(log_contents)
