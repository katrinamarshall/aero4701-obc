import os

import kivy
import paramiko
import rospy
from dismissable_popup import show_dismissable_popup
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import StringProperty
from kivy.uix.popup import Popup

kivy.require("1.2.0")

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("inputPasswordPopup.kv"))


class InputPasswordPopup(Popup):

    _password = None

    _AUTH_FAILED_TEXT = "Authentication failed"
    _password_property = StringProperty()
    _error_property = StringProperty()

    def __init__(
        self,
        ssh_client,
        on_success_func,
        on_failure_func,
        hostname="192.168.2.2",
        username="bot",
        **kwargs
    ):
        """[Popup window to allow the user to input a password to authenticate an ssh_client connection to the bot.

        On authentication the on_success_func function is called and on_failure_func is called if the password
        is incorrect. The password is stored as a static variable so once the password is correctly entered the class
        will open a connection with the client using this client rather than asking the user to enter the password
        every time.]

        Args:
            ssh_client ([paramiko.SSHClient]): [The SSH Client to connect to the bot with. We expect that the options
            set_missing_host_key_policy(paramiko.AutoAddPolicy()) have already been added to the client. ]
            on_success_func ([Callable]): [Function to be called on password success.]
            on_failure_func ([Callable]): [Function to be called on password failure.]
            hostname (str, optional): [The hostname to connect to.]. Defaults to '192.168.2.2'.
            username (str, optional): [Ths username to connect to.]. Defaults to 'bot'.
        """
        super(InputPasswordPopup, self).__init__(**kwargs)
        self._on_success_func = on_success_func
        self._on_failure_func = on_failure_func
        self._host_name = hostname
        self._username = username
        self._ssh_client = ssh_client

    def get_password(self):
        return InputPasswordPopup._password

    def connect(self):
        if InputPasswordPopup._password is None:
            rospy.loginfo("Password not entered. Opening popup")
            self.open()
        else:
            rospy.loginfo("Correct password recoreded. Using previous login details")
            if self._attempt_connection(InputPasswordPopup._password):
                self._on_success_func()
            else:
                self._on_failure_func()

    def _enter(self):
        """[Called when the user presses okay in the Popup window.

        Handles error checking and connection attempts. ]
        """
        if not self.text:
            show_dismissable_popup(
                title=InputPasswordPopup._AUTH_FAILED_TEXT,
                text="Password cannot be none",
                size_x=400,
                size_y=120,
            )
        else:
            password = self.text
            if self._attempt_connection(password):
                # Set password for this session so we dont neeed to enter it again
                InputPasswordPopup._password = password
                self._on_success_func()
                self.dismiss()
            else:
                show_dismissable_popup(
                    title=InputPasswordPopup._AUTH_FAILED_TEXT,
                    text="Password incorrect",
                    size_x=400,
                    size_y=120,
                )
                self._on_failure_func()

    def _cancel(self):
        """[Called when the user presses cancel on the popup window. ]"""
        self.dismiss()

    def _attempt_connection(self, entered_password):
        """[Attempts to connect to the bot using the ssh client and provided username and hostname. ]

        Args:
            entered_password ([str]): [Password to attempt the connection with. ]

        Returns:
            [bool]: [If the connection is successful. ]
        """
        try:
            self._ssh_client.connect(
                hostname=self._host_name,
                username=self._username,
                password=entered_password,
                timeout=10,
            )
            rospy.loginfo("Password success: connection to bot verified")
            return True
        except paramiko.SSHException as e:
            rospy.logwarn("Password failure: {}".format(str(e)))
            return False


if __name__ == "__main__":

    class WidgetApp(App):
        def build(self):
            return InputPasswordPopup()

    WidgetApp().run()
