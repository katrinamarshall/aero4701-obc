#!/usr/bin/env python
import os
import textwrap
from threading import Thread

import kivy
import rosnode
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from hullbot_msgs.srv import SetString
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from MissionNamePopup import MissionNamePopup
from std_srvs.srv import Trigger

from seawolf.utils.ThreadingSupport import threaded

kivy.require("1.2.0")

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("rosbagOptionsPopup.kv"))


class RosbagOptionsPopup(Popup):
    """
    A popup to stop rosbag recording or start a new recording, accessible from the ButtonsPane
    Also contains the _stop_rosbag_recording and _start_rosbag_recording methods used to directly stop
    and start recording
    Starting a new recording will terminate any current recording
    """

    def __init__(self, change_record_func, server=None, **kwargs):
        # Kivy
        super(RosbagOptionsPopup, self).__init__(**kwargs)

        self.server = server
        self.change_record_func = change_record_func
        # Instantiate the MissionNamePopup and give it the function to run when the name is confirmed (i.e. start recording)
        self.MissionNamePopup = MissionNamePopup(on_confirm_func=self._start_rosbag_recording)

        self._rosbag_health_check_timer = None
        self.rosbag_record_start_service = rospy.ServiceProxy("~rosbag_start", SetString)
        self.rosbag_record_stop_service = rospy.ServiceProxy("~rosbag_stop", Trigger)
        self.diagnostics_sub = rospy.Subscriber(
            "~diagnostics", DiagnosticArray, callback=self.diagnostics_cb
        )

        self.is_recording = False
        self.last_heartbeat_time = None

        # These are returned from the rosbag starting service
        self.bag_name_prefix = None
        self.current_rosbag_folder = None
        self.mission_name = None

        self.blue_colour = [0.17, 0.3, 1, 0.5]
        self.grey_colour = [1, 1, 1, 0.8]

        # If a bag has somehow been left recording, kill it for a fresh start
        # This may happen if seawolf did not quit safely
        if "/rosbag/mission/record" in rosnode.get_node_names():
            os.system("rosnode kill /rosbag/mission/record")

    def error_popup(self, title, text):
        # Break up into multiple lines if the message is long, preserving newlines
        wrapped = "\n".join(textwrap.fill(x, 55) for x in text.splitlines())

        popup = Popup(
            title=title, content=Label(text=wrapped), size_hint=(None, None), size=(450, 200)
        )
        popup.open()

    def diagnostics_cb(self, diag_arr_msg):
        if self.is_recording:
            for status in diag_arr_msg.status:
                if status.name == "rosbag_mission_manager":
                    self.last_heartbeat_time = rospy.Time.now().to_sec()
                    if status.level == DiagnosticStatus.WARN:
                        self.error_popup("ROSBAG WARNING", status.message)
                    elif status.level == DiagnosticStatus.ERROR:
                        self.error_popup("ROSBAG ERROR", status.message)
                        # An error indicates that the bag is no longer recording
                        print(status.message)
                        self.is_recording = False
                        if self._rosbag_health_check_timer:
                            self._rosbag_health_check_timer.shutdown()
                            self._rosbag_health_check_timer = None
                            self.last_heartbeat_time = None

    def start_new_recording_button(self):
        """
        Callback for the "Start New Recording" button in the popup
        """
        self.dismiss()
        self.prompt_recording()

    def stop_recording_button(self):
        """
        Callback for the "Stop Recording" button in the popup
        """
        self.stop_rosbag_recording()
        self.dismiss()

    def prompt_recording(self):
        """
        Prompt the user for a name for the new rosbag and then start the recording
        Rosbag recordings should be initiated from this function
        """

        # Stop any current recording if one is already running
        if self.is_recording:
            rospy.loginfo("Stopping current rosbag recording")
            self.stop_rosbag_recording()

        self.MissionNamePopup.open()

    @threaded
    def _start_rosbag_recording(self):
        """
        Send a request to topside core to start rosbag recording
        """

        try:
            self.mission_name = App.get_running_app().config.get(
                "previousmission", "previousmissionname"
            )
        except Exception as e:
            # Note that this should never happen, but we don't want ever to lose rosbag data, so setting a default
            rospy.logerr("Exception when getting mission name from config: {}".format(e))
            self.mission_name = "unknown_mission"
        rospy.loginfo("Rosbag will use mission name: {}".format(self.mission_name))

        try:
            robot_name = App.get_running_app().config.get(
                "previousmission", "previousmissionrobot"
            )
        except Exception as e:
            # Again, should never happen
            rospy.logerr("Exception when getting robot name from config: {}".format(e))
            robot_name = "unknown_robot"
        rospy.loginfo("Rosbag will use robot name: {}".format(robot_name))

        try:
            mission_description = App.get_running_app().config.get(
                "previousmission", "previousmissiondescription"
            )
        except Exception as e:
            # Again, should never happen
            rospy.logerr("Exception when getting robot name from config: {}".format(e))
            mission_description = "unknown_mission_description"
        rospy.loginfo("Rosbag will use mission description: {}".format(mission_description))

        start_request = SetString()
        start_request.data = self.mission_name + "\n" + robot_name + "\n" + mission_description

        try:
            resp = self.rosbag_record_start_service(start_request.data)
        # If the service cannot be called
        except rospy.ServiceException as err:
            self.error_popup(
                "ROSBAG ERROR",
                "Rosbag recording could not be started, service gave exception:\n{}\nIs rosbag_record node running? Try starting the recording again with Rosbag Options > Start New Recording".format(
                    err
                ),
            )
            rospy.logwarn("rosbag_record start service exception: {}".format(err))

        else:
            # If we get a failure code back, warn the user
            if not resp.success:
                self.error_popup(
                    "ROSBAG ERROR",
                    "Rosbag recording could not be started:\n{}".format(resp.message),
                )
            else:
                self.is_recording = True
                self._rosbag_health_check_timer = rospy.Timer(
                    rospy.Duration(5.0), self._check_rosbag_health
                )
                self.current_rosbag_folder, self.bag_name_prefix = os.path.split(resp.message)

    def is_running(self):
        """
        Used to check if a rosbag is being recorded
        """
        return self.is_recording

    @threaded
    def stop_rosbag_recording(self):
        """
        Stop rosbag recording
        """
        if self._rosbag_health_check_timer:
            self._rosbag_health_check_timer.shutdown()
            self._rosbag_health_check_timer = None
            self.last_heartbeat_time = None

        rospy.loginfo("Stopping rosbag recording")
        try:
            resp = self.rosbag_record_stop_service()
            if not resp.success:
                self.error_popup(
                    "ROSBAG ERROR",
                    "Rosbag recording could not be stopped:\n{}".format(resp.message),
                )
            else:
                self.is_recording = False
        except rospy.ServiceException as err:
            self.error_popup(
                "ROSBAG ERROR",
                "Rosbag recording could not be stopped, service gave exception:\n{}\nIs rosbag_record node running?".format(
                    err
                ),
            )
            rospy.logerr("rosbag_record stop service exception: {}".format(err))

    def _check_rosbag_health(self, *args):
        """
        Check if we've received a heartbeat from the rosbag recording node in the last 5 seconds (they should come every 4)
        """
        if self.last_heartbeat_time is not None:
            if rospy.Time.now().to_sec() > self.last_heartbeat_time + 5:
                self.error_popup(
                    "ROSBAG ERROR", "Lost connection to rosbag node, rosbag may not be recording"
                )
                self.is_recording = False
                self._rosbag_health_check_timer.shutdown()
                self._rosbag_health_check_timer = None

    def get_rosbag_name(self):
        """Returns the last rosbag name used

        Returns:
            [str]: Prepended name added to rosbag
        """
        return self.bag_name_prefix

    def get_mission_name(self):
        """
        Returns the mission name, used by the POI logger to put the log in the right folder
        """
        return self.mission_name


if __name__ == "__main__":

    class WidgetApp(App):
        def build(self):
            return RosbagOptionsPopup()

    WidgetApp().run()
