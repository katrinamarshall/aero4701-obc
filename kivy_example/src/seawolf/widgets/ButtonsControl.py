import json
import os
import re
import socket
import subprocess
import time
from socket import errno as socket_errno
from socket import error as socket_err

import actionlib
import Keypress

## Kivy imports are placed after standard library imports
import kivy
import rospy
from geometry_msgs.msg import PoseStamped
from hb_control_msgs.msg import StationKeepingModes
from hullbot_msgs.msg import FeatherCommandAction, FeatherCommandGoal
from hullbot_msgs.srv import SrvResetToPose, SrvResetToPoseRequest
from std_msgs.msg import Bool, Float32, String, UInt8MultiArray
from std_srvs.srv import SetBool, SetBoolRequest, Trigger

kivy.require("1.2.0")
import paramiko

# TODO: for now
import roslaunch
import rospkg
from dismissable_popup import show_dismissable_popup
from hullbot_common.shell_cmd import ShellCmd
from InputPasswordPopup import InputPasswordPopup
from kivy.properties import BooleanProperty, ObjectProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from PointOfInterestLog import PointOfInterestLog
from RosbagOptionsPopup import RosbagOptionsPopup
from SystemTest import SystemTest
from xmlrpclib_timeout import ServerProxy

from seawolf.utils.ThreadingSupport import *
from seawolf.utils.ThreadingSupport import threaded


class LauncherStarter:
    def __init__(self, pkg_name, launcher_name, *args):
        if not args:
            launcher_path = "launch/"
        else:
            launcher_path = args[0]

        rospack = rospkg.RosPack()
        path = rospack.get_path(pkg_name)
        path = path + "/" + launcher_path + launcher_name
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
        self.launch.start()

    def stop(self):
        print("killing launch")
        self.launch.shutdown()


class ButtonsControl:
    """
    This class controls base functionality launch/shutdown of various components as well as rosbag options
    """

    instance = None

    @staticmethod
    def get_instance():
        if ButtonsControl.instance is None:
            ButtonsControl.instance = ButtonsControl()
        return ButtonsControl.instance

    def __init__(self, **kwargs):

        # TODO: Encapsulate the Supervisor calls in a more elegant way (with less code duplication)
        # No need to create the object and re-use it, it's very cheap to create a new one
        # every time we want to make a call, and on every creation we can set a different timeout
        # Currently, it's problematic to share the same timeout because stop operations take longer
        # (because they wait for processes to die) than start operations (only wait for a predefined
        # amount of seconds, in general 3s)
        # Furthermore, starting topside_core may imply starting the container too and the logic
        # is partially mixed together with the supervisor logic right now

        # Get supervisor client for laptop topside
        try:
            self.topside_proc_server = ServerProxy("http://10.10.1.1:9001", timeout=5.0)
        except Exception as e:
            print("Exception when creating ServerProxy to laptop topside: {}".format(e))

        # Get supervisor client for H6 botside
        try:
            self.bot_proc_server = ServerProxy("http://10.10.1.30:9001", timeout=5.0)
        except Exception as e:
            print("Exception when creating ServerProxy to H6 botside: {}".format(e))

        # Check id our IP address 2.1 to see if topside should run locally or on pod
        ip_addresses = subprocess.check_output(
            "ifconfig | grep 'inet ' | grep -v 127.0.0.1 | grep -v 172.* | awk '{print $2}'",
            shell=True,
        )
        if "10.10.1.1" in ip_addresses:
            self.is_topside_local = True
        else:
            self.is_topside_local = False

        # Stores the result of the supervisor call to start the node, used to tell if it's running (on Topside)
        self.rosbag_node_result = None

        # Initialise the rosbag options popup and prompt user to start a rosbag
        self.rosbag_options = RosbagOptionsPopup(
            self.change_rosbag_recording_location,
            self.topside_proc_server,
        )
        self.point_of_interest = PointOfInterestLog()

        # Initialise an instance of KeyHandler to be able to create function commands off key press
        self.keylogger = Keypress.KeyHandler()

        # This is for the keybind for opening the POI window, bound to '`' key
        self.keylogger.bind_key_to_func("`", self.log_point_of_interest, "POI", "open POI")
        self.system_test = SystemTest()

    def on_stop(self):
        if self.rosbag_options.is_running():
            rospy.loginfo("Seawolf quitting, ending rosbag recording")
            self.rosbag_options.stop_rosbag_recording()

    def launch_rviz(self):
        print("Launching Rviz...")
        rviz_launcher = LauncherStarter("h6_bringup", "h6_rviz.launch")

    def launch_plotjuggler(self):
        print("Launching Plotjuggler...")
        plotjuggler_launcher = LauncherStarter("seawolf", "plotjuggler.launch")

    def launch_smach_viewer(self):
        print("Launching Smach Viewer...")
        smach_viewer_launcher = LauncherStarter("seawolf", "smach_viewer.launch")

    def launch_robot_monitor(self):
        print("Launching Robot Monitor...")
        robot_monitor_launcher = LauncherStarter("seawolf", "h6_rqt.launch")

    @threaded
    def start_and_launch_topside(self):
        """
        Start and launch topside.
        If topside is on laptop (local), create/start container then launch topside processes
        If topside is on pod, launch topside processes only as we asssume pod supervisor server is already up
        """
        if self.is_topside_local:
            self.start_topside_container()
            self.launch_topside()
        else:
            self.launch_topside()

    def start_topside_container(self):
        """
        Start the topside container to launch local supervisor server.
        If not created, create the container then start it.
        Additionally check if supervisor server is up as it takes about 30s to create and start a new container.
        """
        # Get the supervisor server name
        server_str = str(self.topside_proc_server)

        # Create topside_core container
        rospy.loginfo(
            "Creating/starting topside_core container and launching topside supervisor server"
        )
        self.topside_container_proc = ShellCmd(
            "/home/user/catkin_ws/src/seawolf/run_topside_container_local.sh"
        )

        # Check supervisor server status until timeout
        rospy.loginfo("Waiting for topside supervisor server to be up")
        timeout = 20
        start = time.time()
        while (
            not self.is_supervisor_server_up(self.topside_proc_server)
            and not rospy.is_shutdown()
            and not self.topside_container_proc.is_done()
        ):
            now = time.time()
            if (now - start) > timeout:
                rospy.logerr("Could not connect to {} within {}s".format(server_str, timeout))
                break
            # Don't spam too quick this loop
            rospy.sleep(1.0)
            rospy.loginfo(
                "Current log from {}:\n{}".format(
                    self.topside_container_proc.cmd, self.topside_container_proc.get_stdout()
                )
            )

        # If the script didn't finish but the supervisor server was already up, wait for the script to finish
        if not self.topside_container_proc.is_done() and self.is_supervisor_server_up(
            self.topside_proc_server
        ):
            self.topside_container_proc.wait_until_finished(timeout=5.0)

        # If the script finished, check why
        if self.topside_container_proc.is_done():

            # If it finished not successfully, show an error
            if not self.topside_container_proc.is_succeeded():
                popup_text = "There was some problem starting topside_core_container locally:"
                popup_text += "\nThe command was: '{}'".format(self.topside_container_proc.cmd)
                popup_text += "\nstdout:\n" + self.topside_container_proc.get_stdout()
                popup_text += "\n\nstderr:\n" + self.topside_container_proc.get_stderr()
                show_dismissable_popup(
                    title="Error starting topside_core container",
                    text=popup_text,
                    size_x=900,
                    size_y=600,
                )
                rospy.logerr(popup_text)
                return

        # If the script didn't finish, it means it started the container with --attach and everything is correct
        rospy.loginfo(
            "Starting topside_core_container log:\n {}".format(
                self.topside_container_proc.get_stdout()
            )
        )

        # Start all topside processes if supervisor server is up
        rospy.loginfo("Supervisor on topside running")

    def launch_topside(self):
        """
        Launch the topside processes via a supervisor call
        """
        # Check if supervisor server for topside_core is alive
        rospy.loginfo("Running topside processes")
        if not self.is_supervisor_server_up(self.topside_proc_server):
            return

        # Check the location of topside server (laptop or pod)
        if self.is_topside_local:
            process_group = "run_common"
        elif not self.is_topside_local:
            process_group = "pod_run_common"

        # Start the process group
        ## Note: unfortunately, and unknowingly why, an exception in this 'try' block, does not trigger
        ## an exception but crashes kivy with no log about why. Somehow kivy is swallowing the exception
        rospy.loginfo("Launching topside processes on process group :{}".format(process_group))
        try:
            results = self.topside_proc_server.supervisor.startProcessGroup(process_group)
            # Log the launch results
            for result in results:
                rospy.loginfo("Process launch result: {}".format(result))
        except Exception as err:
            rospy.logerr("Failed to start topside_core process error {}".format(err))

        self.start_rosbag_node()

        # Give mission manager up to 5 seconds to boot, then check if a systest is needed
        result, message = self.system_test.wait_and_check_needed(5)

        if result is None:
            show_dismissable_popup(
                title="Error",
                text="Could not check last systest time, is mission manager running?\n{}".format(
                    message
                ),
            )
        elif result is True:
            self.system_test.prompt_system_test()

        # Initiate rosbag recording
        rospy.loginfo("Prompting user for rosbag name to start recording")
        self.rosbag_options.prompt_recording()

    def is_supervisor_server_up(self, server):
        """
        Check if the supervisor server is up, first we check if the port 9001 is open
        to avoid possible problems with xmlrpc (kivy seems to swallow some exceptions and just crash
        instead of obeying the try-except block, very weird)
        """
        ip_port = server._ServerProxy__host
        ip, port = ip_port.split(":")
        socket_obj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        port_status = socket_obj.connect_ex((ip, int(port)))

        # If the port is not open, don't bother checking further (usually status 111)
        if port_status != 0:
            return False

        try:
            # Try to read the supervisor server state
            supervisor_state = server.supervisor.getState()
            if supervisor_state["statename"] == "RUNNING":
                # If supervisor server is running, stop blocking
                rospy.logdebug("Connected to supervisor server {}".format(ip_port))
                return True
        except socket_err as err:
            if err.errno == socket_errno.ECONNREFUSED:
                # This is the error for Connection refused, there is no supervisor server running
                rospy.logdebug("Supervisor server not available: {}".format(ip_port))
            else:
                # Some other errors (possibly caused by IP address issues)
                rospy.logdebug(
                    "Supervisor server {sv} connection could not be established with socket error {e}, check your network settings".format(
                        sv=server, e=err
                    )
                )
        except Exception as e:
            rospy.logerr(
                "Got unhandled exception on is_supervisor_server_up ({}): '{}'".format(ip_port, e)
            )
        return False

    @threaded
    def kill_topside(self):
        """
        Kill the topside processes via a supervisor call
        """
        # Check if supervisor server for topside_core is alive
        if not self.is_supervisor_server_up(self.topside_proc_server):
            return
        self.rosbag_options.stop_rosbag_recording()

        # Check the location of topside server (laptop or pod)
        if self.is_topside_local:
            process_group = "run_common"
        elif not self.is_topside_local:
            process_group = "pod_run_common"

        # Stop the process group
        rospy.loginfo("Killing topside processes on process group :{}".format(process_group))
        try:
            results = self.topside_proc_server.supervisor.stopProcessGroup(process_group)
            if results is not None:
                for result in results:
                    rospy.loginfo("Process kill result: {}".format(result))
        except Exception as err:
            # For some reason the logerr doesn't print, but the print does, as for now leaving it here
            # the rospy.logerr can be observed via rqt_console and similar, though
            print(
                "Error when stopping topside_core group on topside, it may have stopped but it took too long. Error: {}".format(
                    err
                )
            )
            rospy.logerr(
                "Error when stopping topside_core group on topside, it may have stopped but it took too long. Error: {}".format(
                    err
                )
            )

        # Stop rosbag recording
        self.stop_rosbag_node()

    def kill_topside_on_exit(self):
        """A simple version of kill_topside that is called on application that doesnt hang for a long time."""

        # Stop current rosbag recording and node.
        self.rosbag_options.stop_rosbag_recording()
        rospy.sleep(1.0)
        self.stop_rosbag_node()

        # Check the location of topside server (laptop or pod)
        if self.is_topside_local:
            process_group = "run_common"
        elif not self.is_topside_local:
            process_group = "pod_run_common"

        try:
            rospy.loginfo("Stopping process group: {}".format(process_group))
            return self.topside_proc_server.supervisor.stopProcessGroup(process_group)
        except Exception as err:
            rospy.logerr("Failed to stop with error {}".format(err))
            return False

    @threaded
    def launch_botside(self):
        """
        Start the botside processes via a supervisor call
        """
        # Check if botside supervisor server is still alive
        if not self.is_supervisor_server_up(self.bot_proc_server):
            rospy.logwarn("No supervisor server detected on botside")
            return

        # Start the process group
        rospy.loginfo("Launching botside processes")
        try:
            results = self.bot_proc_server.supervisor.startProcessGroup("run_common")
            rospy.loginfo("Successfully started process group 'run_common'")
            # Log the results
            if len(results) == 0:
                rospy.loginfo("Seems like all processess were already running")
            for result in results:
                rospy.loginfo("Process launch result: {}".format(result))
        except Exception as err:
            rospy.logerr("Failed to start process with error {}".format(err))

    @threaded
    def kill_botside(self):
        """
        Stop the botside processes via a supervisor call
        """

        # Check if botside supervisor server is still alive
        if not self.is_supervisor_server_up(self.bot_proc_server):
            return

        # Stop the process group
        rospy.loginfo("Killing botside processes")
        try:
            results = self.bot_proc_server.supervisor.stopProcessGroup("run_common")
            if results is not None:
                for result in results:
                    rospy.loginfo("Process kill result: {}".format(result))
        except Exception as err:
            # For some reason the logerr doesn't print, but the print does, as for now leaving it here
            # the rospy.logerr can be observed via rqt_console and similar, though
            print(
                "Error when stopping run_common group on bot, it may have stopped but it took too long. Error: {}".format(
                    err
                )
            )
            rospy.logerr(
                "Error when stopping run_common group on bot, it may have stopped but it took too long. Error: {}".format(
                    err
                )
            )

    def start_rosbag_node(self):
        """
        Start rosbag_record node via a supervisor call
        """
        # Start the process
        try:
            self.rosbag_node_result = self.topside_proc_server.supervisor.startProcess(
                "hb_ui_rosbag_record"
            )
        except Exception as err:
            rospy.logerr("Failed to start topside_core process error {}".format(err))
        else:
            # Log the launch results
            rospy.loginfo(
                "Process launch result: rosbag_record - {}".format(self.rosbag_node_result)
            )

    def stop_rosbag_node(self):
        """
        Stop rosbag_record node via a supervisor call
        """
        # Stop rosbag recording properly
        if self.rosbag_options.is_running():
            self.rosbag_options.stop_rosbag_recording()
            # Make sure the service request can be processed before we kill the node
            rospy.sleep(0.5)

        # Stop the process
        try:
            result = self.topside_proc_server.supervisor.stopProcess("hb_ui_rosbag_record")
            if result is not None:
                rospy.loginfo("Process kill result: hb_ui_rosbag_record - {}".format(result))
        except Exception as err:
            # For some reason the logerr doesn't print, but the print does, as for now leaving it here
            # the rospy.logerr can be observed via rqt_console and similar, though
            print(
                "Error when stopping rosbag_record, it may have stopped but it took too long. Error: {}".format(
                    err
                )
            )
            rospy.logerr(
                "Error when stopping rosbag_record, it may have stopped but it took too long. Error: {}".format(
                    err
                )
            )
        self.rosbag_node_result = None

    def open_rosbag_popup(self):
        self.rosbag_options.open()

    def change_rosbag_recording_location(self, *args):
        """
        Change rosbag_record location

        """
        if self.rosbag_node_result is not None:
            print("Stop topside rosbag_record node")
            self.stop_rosbag_node()
        self.start_rosbag_node()

    def log_point_of_interest(self):
        curr_bag = self.rosbag_options.get_rosbag_name()
        mission_name = self.rosbag_options.get_mission_name()
        if curr_bag is None:
            rospy.logwarn("Cannot create POI, no rosbag found")
            print("Cannot create POI, no rosbag found")
        else:
            self.point_of_interest.update_bag_name(curr_bag, mission_name)
            self.point_of_interest.open()

    class RemoteLogRetriever(object):
        """[A wrapper class to allow easy access to try and retrieve the log files from the remote machine. ]

        Args:
            object ([type]): [description]
        """

        def __init__(self, button_panes):
            self._button_panes = button_panes
            self._ssh_client = paramiko.SSHClient()
            self._ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            # Popup window that allows the user to input the bots password. Handles
            # the authentication of the password and calles the on connection success
            # function on verification and on connection failure if the password
            # is not correct.
            self._input_password = InputPasswordPopup(
                self._ssh_client, self._on_connection_success, self._on_connection_failure
            )

            self._remote_log_path = "/var/log/hullbot_supervisor"
            self._remote_supervisor_zip_path = "/home/bot/supervisor_logs.zip"
            self._local_hullbot_bag_files = "/home/user/Hullbot_bag_files/"

            self._no_remote_log_files_error = (
                ""
                + "There are no log files on the remote system at:\n"
                + "/var/log/hullbot_supervisor"
            )

            self._mission_empty_error = (
                ""
                + "Mission name is empty\n"
                + "Before you retrieve the logs you must start a mission\n"
            )

            self._zipping_logs_error = "" + "Error when trying to zip logs remotely\n"

        def run(self):
            """[Runs the process in order to extract the remote log files from the connected bot. ]"""
            if not self._check_local_folders_exist():
                return

            # Verify after we check the local folder
            self._input_password.connect()
            # run function ends as we wait for _on_connection_sucess callback from the input password popup

        def _monitor_download_progress(self, bytes_transferred, total_bytes):
            """[Callback from the paramiko.SFTPClient during remote transfer that indicates how many bytes
            have been transferred.]

            Args:
                bytes_transferred ([float]): [bytes transferred]
                total_bytes ([float]): [total bytes]
            """
            print("{}/{} bytes".format(bytes_transferred, total_bytes))

            if bytes_transferred == total_bytes:
                rospy.loginfo(
                    "{}/{} bytes transferred. Log file transfer complete".format(
                        bytes_transferred, total_bytes
                    )
                )

        def _on_connection_success(self):
            """[Callback from the RemoteLogRetriever is the password is correct. ]"""
            rospy.logdebug("On connection success")
            if not self._check_remote_logs_exist():
                self._retrieve_logs_error_msg(self._no_remote_log_files_error)
                return
            self._download_remote_logs()

        def _on_connection_failure(self):
            """[Callback from the RemoteLogRetriever if the password input is incorrect. ]"""
            rospy.logwarn("On connection failuire")

        def _check_local_folders_exist(self):
            """[Checks that the mission folder exists on the local machine. If not then we do not let the remote
            logs be retrieved.

            This assumes that the mission folder has been created in the ~/Hullbot_bag_files directory if mission name
            is not None as this is where we will be depositing the logs.
            ]

            Returns:
                [bool]: [If a mission has been started.]
            """
            depositing_folder = self._button_panes.rosbag_options.get_mission_name()
            if depositing_folder == None:
                # Mission has not started so no plac to deposit the logs
                self._retrieve_logs_error_msg(self._mission_empty_error)
                return False
            return True

        def _check_remote_logs_exist(self):
            """[Checks that the remote logs exist in the folder /var/log/hullbot_supervisor.

            Must happen after we verify the connection to the bot.
            ]

            Returns:
                [bool]: [If the remote logs exist. ]
            """
            rospy.loginfo("Checking remote logs exist")
            _, stdout, _ = self._ssh_client.exec_command("ls {}".format(self._remote_log_path))
            files = stdout.readlines()
            rospy.loginfo("Files {}".format(files))
            if len(files) == 0:
                print(
                    "There are no remote logs in the folder {} or the folder does not exist!".format(
                        self._remote_log_path
                    )
                )
                rospy.logwarn(
                    "There are no remote logs in the folder {} or the folder does not exist!".format(
                        self._remote_log_path
                    )
                )
                return False

            return True

        def _construct_remote_files_paths(self, files_list, folder_path):
            """[Construct the full file path to the all the remote logs on the bot.]

            Args:
                files_list ([List[str]]): [List of all the remote file names (not paths) to retrieve. ]
                folder_path ([str]): [Remote folder path where all the files are stored. ]

            Returns:
                [List[str]]: [List containing the full file paths to each log on the remote machine. ]
            """
            remote_file_paths = []
            for unicode_file in files_list:
                file = unicode_file.encode("utf-8")
                file = file.strip()
                remote_file_paths.append(folder_path + "/" + file)

            return remote_file_paths

        @threaded
        def _download_remote_logs(self):
            """[Accses the remote bot using the ssh client and actually downloads the logs from /home/bot/supervisor_logs.zip.
            The logs from /var/log/hullbot_supervisor get zipped here due to permission issues when using the ssh_client
            that prevent us from making a new folder in /var/log]

            """
            if not self._zip_remote_logs():
                self._retrieve_logs_error_msg(self._zipping_logs_error)
                return

            rospy.loginfo(
                "Downloading log ({}) to local: {}".format(
                    self._remote_supervisor_zip_path, self._consruct_local_folder_path()
                )
            )
            transport = self._ssh_client.get_transport()
            sftp_client = paramiko.SFTPClient.from_transport(transport)
            sftp_client.get(
                self._remote_supervisor_zip_path,
                self._consruct_local_folder_path(),
                callback=self._monitor_download_progress,
            )

        def _consruct_local_folder_path(self):
            """[Creates local path where the remote logs.zip file should be copied to on the local machine.

            Will be in the form ~/Hullbot_bag_files/<mission_name>/supervisor_logs.zip]

            Returns:
                [str]: [Full file path for the local log zip file]
            """
            return "{}/{}/supervisor_logs.zip".format(
                self._local_hullbot_bag_files, self._button_panes.rosbag_options.get_mission_name()
            )

        def _zip_remote_logs(self):
            """[Zip's the all the files held in the remote logs folder (/var/log/hullbot_supervisor)
            and puts them into the /home/bot directory. This assumes we have already checked that
            the folders in /var/log/ exist.

            We put them in the /home/bot dir due to access issues when trying to make a new folder on the bot]

            Returns:
                [bool]: [If the remote command was executed successfully. ]
            """
            _, stdout, stderr = self._ssh_client.exec_command(
                "cd {} && zip -r {} .".format(
                    self._remote_log_path, self._remote_supervisor_zip_path
                )
            )
            stderr.readlines()
            length = len(stderr.readlines())
            rospy.sleep(1)
            if length == 0:
                return True
            else:
                print("Error when zipping")
                rospy.logerr("Error when zipping files\n{}".format(stderr.readlines()))
                return False

        def _retrieve_logs_error_msg(self, msg):
            """[Opens a popup window with a custom mesage and the title "Error when retrieving logs"]

            Args:
                msg ([str]): [Custom display message]
            """
            button = Button(
                size_hint=(None, None),
                size=(80, 40),
                text="Got It!",
            )
            popup_content = BoxLayout(orientation="vertical")
            popup_content.add_widget(Label(text=msg))
            popup_content.add_widget(button)
            popup = Popup(
                title="Error when retrieving logs",
                content=popup_content,
                size_hint=(None, None),
                size=(700, 400),
            )
            button.bind(on_release=popup.dismiss)

            popup.open()

    def retrieve_remote_logs(self):
        """
        [Attempts to retrieve all logs from the bot from /log/var/hullbot_supervisor and
        packages them in a .tar in the folder /home/user/Hullbot_bag_files/<mission_name>/supervisor_logs.zip]
        """
        log_retriever = self.RemoteLogRetriever(self)
        log_retriever.run()


class ModeControl:
    """
    This controls autonomous various modes that the robot can be put into
    """

    instance = None

    @staticmethod
    def get_instance():
        # Singleton method to ensure only one instances of the class exists
        if ModeControl.instance is None:
            ModeControl.instance = ModeControl()
        return ModeControl.instance

    def __init__(self, **kwargs):
        # Services
        self.send_toggle_auto_mode = rospy.ServiceProxy(
            "/hullbot/state_machine/toggle_auto_mode", SetBool
        )
        self.enable_tof_z_hold_client = rospy.ServiceProxy(
            "/hullbot/control_mode_handler/enable_tof_z_hold", SetBool
        )
        self.horizon_hold_srv_client = rospy.ServiceProxy(
            "/hullbot/control_mode_handler/enable_horizon_hold", SetBool
        )
        self.auto_mode = BooleanProperty()

    @threaded
    def enable_tof_hold(self):
        req = SetBoolRequest()
        req.data = True
        try:
            resp = self.enable_tof_z_hold_client(req)
            rospy.loginfo("tof hold enable response: {}".format(resp.message))
        except Exception as e:
            print("Exception: {}".format(e))

    @threaded
    def disable_tof_hold(self):
        req = SetBoolRequest()
        req.data = False
        try:
            resp = self.enable_tof_z_hold_client(req)
            rospy.loginfo("tof hold disable response: {}".format(resp.message))
        except Exception as e:
            print("Exception: {}".format(e))

    @threaded
    def enable_horizon_hold(self):
        req = SetBoolRequest()
        req.data = True
        try:
            resp = self.horizon_hold_srv_client.call(req)
            rospy.loginfo("horizon hold enable response: {}".format(resp.message))
        except Exception as e:
            print("Exception: {}".format(e))

    @threaded
    def disable_horizon_hold(self):
        req = SetBoolRequest()
        req.data = False
        try:
            resp = self.horizon_hold_srv_client.call(req)
            rospy.loginfo("horizon hold disable response: {}".format(resp.message))
        except Exception as e:
            print("Exception: {}".format(e))

    @threaded
    def toggle_auto_mode(self):
        try:
            resp = self.send_toggle_auto_mode(SetBoolRequest(not self.auto_mode))
            rospy.loginfo("toggle_auto_mode response: {}".format(resp))
        except Exception as e:
            print("Exception: {}".format(e))


class RovioControl:
    """
    This is a controller for rovio functionality
    """

    instance = None

    @staticmethod
    def get_instance():
        # Singleton method to ensure only one instances of the class exists
        if RovioControl.instance is None:
            RovioControl.instance = RovioControl()
        return RovioControl.instance

    def start_rovio(self):
        """
        Start rovio using remote supervisor
        """
        os.system("nohup supervisorctl -s http://10.10.1.30:9001 restart rovio &")


class StateQueueControl:
    """
    This controls launchng and modifying state queues
    """

    instance = None
    setStateQueueNameFunctions = []

    @staticmethod
    def get_instance(set_state_queue_name_function=None):
        # Singleton method to ensure only one instances of the class exists
        if StateQueueControl.instance is None:
            StateQueueControl.instance = StateQueueControl()
        if set_state_queue_name_function:
            StateQueueControl.setStateQueueNameFunctions.append(set_state_queue_name_function)
        return StateQueueControl.instance

    def __init__(self, **kwargs):
        # Services
        self.clear_state_queue_service = rospy.ServiceProxy(
            "/hullbot/state_machine/clear_state_queue", Trigger
        )

        # publish to necessary ROS topics
        self.states_to_append_pub = rospy.Publisher(
            "/hullbot/state_machine/states_to_append", String, queue_size=None
        )

        self.grid_widgets = []  # stores label widgets used for displaying the StateQueue
        self.state_queue = []
        self.current_state_str = ""  # stores the whole curr state dict as a string for comparison
        self.state_queue_copy = (
            []
        )  # variable which stores the instance of the previous state_queue.

    @threaded
    def clear_state_queue(self):
        try:
            resp = self.clear_state_queue_service()  # Trigger clear state queue service
            rospy.loginfo("clear_state_queue response: {}".format(resp))
        except Exception as e:
            print("clear_state_queue service call exception: {}".format(e))

    def load_states_to_queue(self, filename):
        """takes the filename path of a json script representing the state queue,
        and publishes the equivalent json string (dict) to a ROS topic
        """
        print("filename: ", type(filename), filename[0])
        loaded_auto_script_abs = filename[0]

        try:
            with open(loaded_auto_script_abs, "r") as f:  # open auto script
                state_list_to_append = json.load(f)  # load it as a json dict
                state_list_to_append = json.dumps(state_list_to_append)  # dump it as a json str
            self.states_to_append_pub.publish(state_list_to_append)
            self.dismiss_popup()
        except Exception as error:
            rospy.logerr("Error opening loaded script: {}".format(error))

        for function in StateQueueControl.setStateQueueNameFunctions:
            function(loaded_auto_script_abs)

    def dismiss_popup(self):
        self._popup.dismiss()

    def show_load(self):
        content = StateQueueControl.LoadDialog(
            load=self.load_states_to_queue, cancel=self.dismiss_popup
        )
        self._popup = Popup(title="Load file", content=content, size_hint=(0.9, 0.9))
        self._popup.open()

    class LoadDialog(FloatLayout):
        load = ObjectProperty(None)
        cancel = ObjectProperty(None)
        auto_scripts_path = StringProperty("/home/user/catkin_ws/src/seawolf/auto_scripts")

        def __init__(self, **kwargs):
            super(StateQueueControl.LoadDialog, self).__init__(**kwargs)


class LightControl:
    """
    Controller for HBLED lights
    """

    instance = None

    @staticmethod
    def get_instance():
        # Singleton method to ensure only one instances of the class exists
        if LightControl.instance is None:
            LightControl.instance = LightControl()
        return LightControl.instance

    def __init__(self):
        self.lights_command_pub = rospy.Publisher(
            "~lights_brightness", UInt8MultiArray, queue_size=1
        )

        # Light config for VAL bot
        self.val_label = "VAL"
        self.val_light_groups = {
            "top": LightGroup(False, [0, 7, 8, 15]),
            "glow": LightGroup(False, [3, 4, 11, 12]),
            "front": LightGroup(False, [5, 10]),
            "side": LightGroup(False, [1, 2, 6, 9, 13, 14]),
        }

        # Light config for SOPH bot
        self.soph_label = "SOPH"
        self.soph_light_groups = {
            "top": LightGroup(False, [0, 3, 12, 15]),
            "glow": LightGroup(False, [1, 2, 13, 14]),
            "front": LightGroup(False, [4, 5, 8, 9]),
            "side": LightGroup(False, [6, 7, 10, 11]),
        }

        self.selected_light_groups = self.val_light_groups
        self.selected_label = self.val_label

        self.reset_light_command()

    def reset_light_command(self):
        self.light_command = len(
            self.selected_light_groups["top"].get_indexes()
            + self.selected_light_groups["glow"].get_indexes()
            + self.selected_light_groups["front"].get_indexes()
            + self.selected_light_groups["side"].get_indexes()
        ) * [0]

    def set_light_group_brightness(self, light_group, val):
        """Set light group to desired brightness and publish command.

        Args:
            light_group (str): light group name i.e top, glow etc. (Must be valid LightGroup object name).
            val (int): brightness value [0 - 255].
        """
        lights = self.selected_light_groups.get(light_group)

        # Set new brightness setpoint
        lights.set_brightness_setpoint(val)

        # Set command based on light group and publish
        self.set_command(light_group)
        self.publish_command()

    def set_light_group_state(self, light_group, state):
        """Set light group state to either true or false and publish command.

        Args:
            light_group (str): light group name i.e top, glow etc. (Must be valid LightGroup object name).
        """
        lights = self.selected_light_groups.get(light_group)

        # Toggle light group state
        lights.set_state(state)

        # Set command based on light group and publish
        self.set_command(light_group)
        self.publish_command()

    def set_command(self, light_group):
        """Set certain elements of light command (UInt8MultiArray) based on light group and brightness.

        Args:
            light_group (str): light group name i.e top, glow etc. (Must be valid LightGroup object name).
            val (int): brightness value [0 - 255].
        """

        lights = self.selected_light_groups.get(light_group)

        for idx in lights.get_indexes():
            if lights.get_state():
                self.light_command[idx] = lights.get_brightness_setpoint()
            else:
                self.light_command[idx] = 0

    def get_light_group_state(self, light_group):
        """Get light group state.

        Args:
            light_group (str): light group name i.e top, glow etc. (Must be valid LightGroup object name).

        Returns:
            bool: _description_
        """
        lights = self.selected_light_groups.get(light_group)
        return lights.get_state()

    def publish_command(self):
        """Publish light command to ROS."""
        self.lights_command_pub.publish(UInt8MultiArray(data=self.light_command))

    def toggle_selected_light_type(self):
        """Set selected light type to match different physical light configuration.

        This updates the light grouping indexes and label for type indication.
        """

        # Toggle selected light type between VAL and SOPH bot
        if self.selected_light_groups == self.val_light_groups:
            self.selected_light_groups = self.soph_light_groups
            self.selected_label = self.soph_label
        elif self.selected_light_groups == self.soph_light_groups:
            self.selected_light_groups = self.val_light_groups
            self.selected_label = self.val_label

        # Reset light command and publish
        self.reset_light_command()
        self.publish_command()

        return self.selected_label

    def get_selected_light_type(self):
        return self.selected_label


class LightGroup:
    def __init__(self, initial_state, indexes):
        self._state = initial_state
        self._indexes = indexes
        self._brightness_setpoint = 0

    def get_state(self):
        return self._state

    def get_indexes(self):
        return self._indexes

    def get_brightness_setpoint(self):
        return self._brightness_setpoint

    def set_state(self, state):
        self._state = state

    def set_brightness_setpoint(self, val):
        self._brightness_setpoint = val


class PodControl:
    instance = None
    pod_calibration_result_callbacks = []
    tether_speed_min = -0.25
    tether_speed_max = 0.25

    def __init__(self):
        self.calibrate_spring_service = rospy.ServiceProxy(
            "/hullbot/pod/torsion_spring_calibration", Trigger
        )
        self.motor_switch_service = rospy.ServiceProxy("/hullbot/pod/motor_on", SetBool)
        self.tether_speed_pub = rospy.Publisher(
            "/hullbot/pod/set_tether_speed", Float32, queue_size=1
        )
        self.pod_command_pub = rospy.Publisher(
            "/hullbot/pod/feather/command", String, queue_size=1
        )

        # Feather
        self.feather_action_client = None
        self.timeout = 5

        # Upon starting, motors are off so spring calibration should not be possible
        self.spring_calibrated = False
        self.motors_on = False

        # Initially tether not moving
        self.current_tether_speed = 0.0

    @staticmethod
    def get_instance(pod_calibration_result_callback):
        # Singleton method to ensure only one instances of the class exists
        if PodControl.instance is None:
            PodControl.instance = PodControl()
        PodControl.pod_calibration_result_callbacks.append(pod_calibration_result_callback)
        return PodControl.instance

    def get_feather_action_client(self):
        # Wait for feather command action server
        self.feather_action_client = actionlib.SimpleActionClient(
            "feather_command", FeatherCommandAction
        )
        rospy.loginfo(
            "Waiting for feather_command server [Server timeout: {} s]".format(self.timeout)
        )
        feather_server_connection = self.feather_action_client.wait_for_server(
            timeout=rospy.Duration(self.timeout)
        )
        if feather_server_connection:
            rospy.loginfo("Connected to feather_command server")
        else:
            rospy.loginfo("Unable connect to feather_command server")

    def emergency_stop(self, msg):
        self.tether_speed_pub.publish(Float32(0.0))

    @threaded
    def calibrate_spring(self):
        rospy.loginfo("Calibrating Torsion Spring")

        # Should not be moving during calibration
        self.tether_speed_pub.publish(Float32(0.0))

        # Without sleep, the motors don' get enough time to stop before the spring calibrates
        rospy.sleep(1.0)

        try:
            result = self.calibrate_spring_service()

            # Check service didn't return a fail
            if result.success:
                self.spring_calibrated = True
                rospy.loginfo("Successfully calibrated torsion spring")
                self.send_calibration_result(True)

            else:
                rospy.logerr("Calibrate spring service did not process request")
                show_dismissable_popup(
                    title="Torsion Spring Calibration Failed", text=result.message
                )
                self.send_calibration_result(False)

        except rospy.ServiceException as e:
            rospy.logerr("Calibrate spring service did not process request")
            show_dismissable_popup(title="Torsion Spring Calibration Failed", text=str(e))
            self.send_calibration_result(False)

    def send_calibration_result(self, result):
        for callback in PodControl.pod_calibration_result_callbacks:
            callback(result)

    def send_pod_command(self, command):
        if self.feather_action_client is None:
            self.get_feather_action_client()

        if self.feather_action_client is not None:
            rospy.loginfo("Sending command to pod: {}".format(command))
            goal = FeatherCommandGoal(command=str(command))
            self.feather_action_client.send_goal(goal)

        rospy.loginfo("Sending command to pod: {}".format(command))
        command_msg = String()
        command_msg.data = command
        self.pod_command_pub.publish(command_msg)

    def pod_power_switch_callback(self, active, *args):
        if active:
            command = "POD_ON"
        else:
            command = "POD_OFF"

        self.send_pod_command(command)

    def misc_pod_commands_callback(self, command):
        self.send_pod_command(command)

    def motor_switch_callback(self, active, *args):

        req = SetBoolRequest()
        req.data = active

        try:
            resp = self.motor_switch_service(req)
            rospy.loginfo(resp.message)

            # For future reference, this is always returning a True
            if resp.success:
                self.motors_on = active

            else:
                rospy.logerr("Motor switch service did not process request. Is winch power on?")

        except rospy.ServiceException as e:
            rospy.logerr("motor_switch service did not process request: " + str(e))
            show_dismissable_popup(
                title="Toggling Motors", text="Failed to toggle motor state. Are the motors armed?"
            )

        # Spring calibration and the tether cannot work without the motors on
        if not self.motors_on:
            self.tether_speed_pub.publish(Float32(0.0))

        return self.motors_on

    def limit_tether_speed(self, value):
        """
        Limit tether speed to min and max values set for tether slider
        """
        if value > self.tether_speed_max:
            value = self.tether_speed_max
        elif value < self.tether_speed_min:
            value = self.tether_speed_min
        return value

    def tether_slider_callback(self, raw_value, *args):
        """
        Callback when tether speed slider is updated or value is updated via text input
        A float will be passed when updated from the slider, and a string is passed when
        updating from the text input box
        """

        # Filter text input to ensure it is a float
        try:
            value = float(raw_value)
            value = round(value, 2)
        except ValueError:
            rospy.logwarn("Invalid tether speed input.  Must be floating point value")
            return False

        # Prevents callback from publishing same message multiple times
        if self.current_tether_speed != value:

            # Limit value to min/max set for tether speed slider
            value = self.limit_tether_speed(value)

            # Publish tether speed
            self.tether_speed_pub.publish(Float32(value))
            self.current_tether_speed = value

        return True


class SlamControl:
    instance = None
    region_values_callbacks = []
    dirty_regions_complete_callbacks = []

    def __init__(self):
        self.queue_control = StateQueueControl.get_instance()

        self.find_dirty_regions_service = rospy.ServiceProxy(
            "/hullbot/octomap/find_fouling_regions_srv", Trigger
        )

    @staticmethod
    def get_instance(update_region_values_callback, dirty_regions_complete_callback=None):
        # Singleton method to ensure only one instances of the class exists
        if SlamControl.instance is None:
            SlamControl.instance = SlamControl()
        if dirty_regions_complete_callback:
            SlamControl.dirty_regions_complete_callbacks.append(dirty_regions_complete_callback)
        if update_region_values_callback:
            SlamControl.region_values_callbacks.append(update_region_values_callback)
        return SlamControl.instance

    def reset_octomap(self):
        """
        Starts octomap if not running, otherwise restarts the node
        """
        os.system("nohup supervisorctl -s http://10.10.1.1:9001 restart slam:start_octomap &")

    def region_selector_pressed(self):
        """
        Bound to region select button
        """
        return

    @threaded
    def find_dirty_regions(self):
        """
        Calls service to find all dirty regions in clean-dirty map
        """
        try:
            resp = self.find_dirty_regions_service()  # Trigger find dirty region service
            rospy.loginfo("Find dirty regions response: {}".format(resp))
            # Update region list
            if resp.success:
                n = int(resp.message)
                nums = []
                for i in range(n):
                    nums.append(str(i + 1))
                for callback in SlamControl.region_values_callbacks:
                    callback(nums)
        except Exception as e:
            rospy.logerr("find_dirty_regions service call exception: {}".format(e))

        for callback in SlamControl.dirty_regions_complete_callbacks:
            callback()

    @threaded
    def clean_region(self, region_id_text):
        """
        Calls service to calculate cleaning trajectory for dirty region and loads into state queue
        """
        region_id = -1

        try:
            region_id = int(region_id_text)
        except:
            rospy.logwarn("Invalid region id selected: {}".format(region_id_text))
            return

        rospy.loginfo("Clean region {}".format(region_id))

        # Set region id in autoscript params
        output = ""
        with open(
            "/home/user/catkin_ws/src/seawolf/auto_scripts/ml_dynamic_clean_map.json", "rw"
        ) as file:
            for l in file.readlines():
                # Update region_id in json file
                if "region_id" in l:
                    repl = 'region_id": ' + str(region_id)
                    l = re.sub('region_id"\: .*', repl, l)
                output = output + l

        file.close()
        fout = open("/home/user/catkin_ws/src/seawolf/auto_scripts/ml_dynamic_clean_map.json", "w")
        fout.write(output)
        fout.close()

        # Load state queue
        self.queue_control.load_states_to_queue(
            ["/home/user/catkin_ws/src/seawolf/auto_scripts/ml_dynamic_clean_map.json"]
        )


class ProcessControl:
    """
    Controls related to H6 process launching.
    """

    instance = None

    @staticmethod
    def get_instance():
        # Singleton method to ensure only one instances of the class exists
        if ProcessControl.instance is None:
            ProcessControl.instance = ProcessControl()
        return ProcessControl.instance

    def __init__(self):
        try:
            self.bot_supervisor_server = ServerProxy("http://10.10.1.30:9001", timeout=5.0)
        except Exception as e:
            rospy.loginfo("Exception when creating ServerProxy to H6 botside: {}".format(e))

        self.socket_can_process = "boot:hb_drivers_socketcan_bridge"
        self.enable_tmescs_process = "boot:hb_drivers_enable_tmescs"
        self.disable_tmescs_process = "hb_drivers_disable_tmescs"
        self.thruster_cores_process = "run_common:hb_drivers_thruster_cores"
        self.cleaning_cores_process = "run_common:hb_drivers_cleaning_cores"
        self.inspection_camera_process = "hb_drivers_inspection_cam_4k_30fps"

        self.motion_controller_process = "run_common:hb_control_motion_controllers"
        self.perception_camera_process = (
            "run_common:hb_localisation_perception_cameras_top_left_front_left"
        )
        self.imu_process = "run_common:hb_drivers_imu"

        self.reset_neutral_function = None
        self.reset_true_function = None
        self.reset_warn_function = None

    @threaded
    def reset_tmescs(self):
        # Set indictator to warning
        self.reset_warn_function()

        # Stop socket can to disable CAN communicaton
        self.stop_socket_can()
        rospy.sleep(1.0)

        # Restart tmescs and wait approx 20s for tmesc to boot
        self.restart_tmescs()
        rospy.sleep(20.0)

        # Start socket can for CAN communication
        self.start_socket_can()
        rospy.sleep(3.0)

        # Restart thruster and cleaning cores to re-initialise tmesc config
        self.restart_thruster_cores()
        self.restart_cleaning_cores()

        # Set indicator to true then back to neutral
        self.reset_true_function()
        rospy.sleep(3.0)
        self.reset_neutral_function()
        return True

    def register_reset_neutral(self, func):
        self.reset_neutral_function = func

    def register_reset_true(self, func):
        self.reset_true_function = func

    def register_reset_warn(self, func):
        self.reset_warn_function = func

    def start_socket_can(self):
        self._start_process(self.socket_can_process)

    def stop_socket_can(self):
        self._stop_process(self.socket_can_process)

    def restart_socket_can(self):
        self._restart_process(self.socket_can_process)

    def enable_tmescs(self):
        self._start_process(self.enable_tmescs_process)

    def disable_tmescs(self):
        self._start_process(self.disable_tmescs_process)

    def restart_tmescs(self):
        self.disable_tmescs()
        rospy.sleep(5.0)
        self.enable_tmescs()

    def start_thruster_cores(self):
        self._start_process(self.thruster_cores_process)

    def stop_thruster_cores(self):
        self._stop_process(self.thruster_cores_process)

    def restart_thruster_cores(self):
        self._restart_process(self.thruster_cores_process)

    def start_cleaning_cores(self):
        self._start_process(self.cleaning_cores_process)

    def stop_cleaning_cores(self):
        self._stop_process(self.cleaning_cores_process)

    def restart_cleaning_cores(self):
        self._restart_process(self.cleaning_cores_process)

    def restart_motion_controllers(self):
        self._restart_process(self.motion_controller_process)

    def restart_perception_cameras(self):
        self._restart_process(self.perception_camera_process)

    def restart_imu(self):
        self._start_process(self.imu_process)

    @threaded
    def start_inspection_camera(self):
        self._start_process(self.inspection_camera_process)

    @threaded
    def stop_inspection_camera(self):
        self._stop_process(self.inspection_camera_process)

    @threaded
    def fix_socketcan_crash(self):
        self.restart_socket_can()
        rospy.sleep(1.0)
        self.restart_thruster_cores()
        self.restart_cleaning_cores()

    @threaded
    def fix_network_dropout(self):
        self.restart_motion_controllers()
        self.restart_perception_cameras()

    @threaded
    def fix_imu_dropout(self):
        self.restart_imu()

    def _start_process(self, process_name):
        try:
            rospy.loginfo("Starting process: {}".format(process_name))
            return self.bot_supervisor_server.supervisor.startProcess(process_name)
        except Exception as err:
            rospy.logerr("Failed to start with error {}".format(err))
            return False

    def _stop_process(self, process_name):
        try:
            rospy.loginfo("Stopping process: {}".format(process_name))
            return self.bot_supervisor_server.supervisor.stopProcess(process_name)
        except Exception as err:
            rospy.logerr("Failed to stop with error {}".format(err))
            return False

    def _restart_process(self, process_name):
        self._stop_process(process_name)
        self._start_process(process_name)


class CleaningControl:
    """
    Controls related to H6 cleaning.
    """

    instance = None

    @staticmethod
    def get_instance():
        # Singleton method to ensure only one instances of the class exists
        if CleaningControl.instance is None:
            CleaningControl.instance = CleaningControl()
        return CleaningControl.instance

    def __init__(self):
        self._joust_client = rospy.ServiceProxy("~cleaning_joust", SetBool)

    def enable_joust(self):
        self._set_joust(True)

    def disable_joust(self):
        self._set_joust(False)

    def _set_joust(self, req):
        rospy.loginfo("Joust: {}".format(req))
        try:
            self._joust_client(req)
        except rospy.ServiceException as err:
            rospy.logerr("Joust service call failed: " + str(err))


class TurboControl:
    """
    Controls related to H6 Turbo Mode.
    Turbo Mode increases the maximium velocity limits set in the joystick translator.
    """

    instance = None

    @staticmethod
    def get_instance():
        # Singleton method to ensure only one instances of the class exists
        if TurboControl.instance is None:
            TurboControl.instance = TurboControl()
        return TurboControl.instance

    def __init__(self):
        self._turbo_mode_client = rospy.ServiceProxy("~turbo_mode", SetBool)

    def enable_turbo_mode(self):
        self._set_turbo_mode(True)

    def disable_turbo_mode(self):
        self._set_turbo_mode(False)

    def _set_turbo_mode(self, req):
        rospy.loginfo("Turbo Mode: {}".format(req))
        try:
            self._turbo_mode_client(req)
        except rospy.ServiceException as err:
            rospy.logerr("Turbo Mode service call failed: " + str(err))


class DepthLockControl:
    """
    Controls related to depth lock
    """

    instance = None

    @staticmethod
    def get_instance(*args):
        # Singleton method to ensure only one instances of the class exists
        if DepthLockControl.instance is None:
            DepthLockControl.instance = DepthLockControl(*args)
        return DepthLockControl.instance

    def __init__(self, text_input):
        self._depth_increment = 0.1
        self._setpoint = None
        self._text_input = text_input

        # Default setpoint message to set all axes except the depth as NaN to keep them unchanged
        self._pose_msg = PoseStamped()
        self._pose_msg.header.frame_id = "world"
        self._pose_msg.pose.position.x = float("nan")
        self._pose_msg.pose.position.y = float("nan")
        self._pose_msg.pose.orientation.x = float("nan")
        self._pose_msg.pose.orientation.y = float("nan")
        self._pose_msg.pose.orientation.z = float("nan")
        self._pose_msg.pose.orientation.w = float("nan")

        self._setpoint_pub = rospy.Publisher("~station_keeping_setpoint", PoseStamped)
        self._control_mode_sub = rospy.Subscriber(
            "~station_keeping_control_modes", StationKeepingModes, callback=self.control_mode_cb
        )
        self._setpoint_sub = rospy.Subscriber(
            "~station_keeping_current_setpoint", PoseStamped, callback=self.setpoint_sub_cb
        )

    def control_mode_cb(self, msg):
        """Store station keeping state and disable input if not in a Depth Lock mode

        Args:
            msg (StationKeepingModes): msg
        """
        # If we go into a depth lock state, enable the text input
        if msg.z_mode == StationKeepingModes.LOCK:
            self._text_input.disabled = False
        else:
            self._text_input.disabled = True

    def setpoint_sub_cb(self, msg):
        """Track the latest Station Keeping setpoint

        Args:
            msg (_type_): _description_
        """
        self._setpoint = abs(msg.pose.position.z)
        if self._text_input.focus == False:
            self._text_input.text = "{:.2f}".format(self._setpoint)

    def setpoint_increment(self):
        """Publish a setpoint incremented on button press"""
        print(self._text_input)
        if not self._text_input.readonly and self._setpoint is not None:
            self.pub_setpoint(self._setpoint - self._depth_increment)

    def setpoint_decrement(self):
        """Publish a setpoint decremented on button press"""
        if not self._text_input.readonly and self._setpoint is not None:
            self.pub_setpoint(self._setpoint + self._depth_increment)

    def on_enter(self, text):
        """On confirmation of the depth input (pressing enter)

        Args:
            text (_type_): the input depth setpoint
        """
        # Negative because we've constrained our input to be positive but we actually want to be
        self.pub_setpoint(float(self._text_input.text))

    def pub_setpoint(self, depth):
        self._pose_msg.pose.position.z = -abs(depth)
        self._pose_msg.header.stamp = rospy.Time.now()
        self._setpoint_pub.publish(self._pose_msg)
