import datetime
import os
from threading import Thread

import actionlib
import kivy
import rospy

kivy.require("1.2.0")
from dismissable_popup import show_dismissable_popup
from hullbot_common.quat_calc import quat_to_eul_deg
from hullbot_msgs.msg import (
    PowerBoardADC,
    TestCCsAction,
    TestCCsGoal,
    TestIMUAction,
    TestIMUGoal,
    TestIMUResult,
    TestTCsAction,
    TestTCsGoal,
    TestTOFsAction,
    TestTOFsGoal,
)
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ListProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.popup import Popup
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Int8, Int16MultiArray

import seawolf
from seawolf.utils.ThreadingSupport import threaded

Builder.load_file(seawolf.form_kivy_config_path("motorTesterPane.kv"))

TEST_STATUS_SUCCESS = 0
TEST_STATUS_WARNING = 1
TEST_STATUS_FAILURE = 2


class SystemIndicator:
    """
    This is an indicator for the status of a particular system (eg. TOFs)
    """

    action_topic = StringProperty("")  # action server topic
    action_type = None
    action_goal = None
    component_label = []
    ac = None
    working = False

    def __init__(self, action_topic, label, action_type, action_goal):
        self.action_topic = action_topic
        self.label = label
        self.action_type = action_type
        self.action_goal = action_goal
        self.status = TEST_STATUS_FAILURE

    def create_action_client(self):
        self.ac = actionlib.SimpleActionClient(self.action_topic, self.action_type)

    def setComponentLabels(self, labels):
        self.component_label = labels

    def setActionType(self, act_type):
        self.action_type = act_type

    def setActionGoal(self, goal):
        self.action_goal = goal

    def getActionTopic(self):
        return self.action_topic

    def getSystemLabel(self):
        return self.label

    def getActionType(self):
        return self.action_type

    def getActionGoal(self):
        return self.action_goal

    def set_status(self, status):
        if status >= 0 and status <= 3:
            self.status = status


class RunSystemsTest(BoxLayout):
    statusColour = ListProperty([1, 1, 1, 0.4])  # set grey by default
    label = StringProperty("")  # the label on the indicator

    def __init__(self, **kwargs):
        super(RunSystemsTest, self).__init__(**kwargs)

    def set_label(self, new_label):
        self.label = new_label


class SystemTestPane(BoxLayout):
    def __init__(self, button=None, **kwargs):
        super(SystemTestPane, self).__init__(**kwargs)
        self.indicator_container = self.ids.indicator_display
        self.systems_list = []
        self.now_testing = 0

        self.status_pub = rospy.Publisher("/hullbot/system_test_status", Int8, queue_size=1)
        rospy.Subscriber("/Hullbot/bridge/heartbeat/state", State, self.state_cb)

        self.system_idx = {"CC": 0, "TOF": 1, "TC": 2, "IMU": 3}
        system_test_action = [
            "/hullbot/cleaning_cores/test_cc",
            "/hullbot/tofs_node/test_tofs",
            "/hullbot/thruster_cores/test_tc",
            "/hullbot/pixhawk_imu/test_imu",
        ]
        system_labels = ["CC", "TOF", "TC", "IMU"]
        system_action_type = [
            type(TestCCsAction()),
            type(TestTOFsAction()),
            type(TestTCsAction()),
            type(TestIMUAction()),
        ]
        system_action_goal = [TestCCsGoal(), TestTOFsGoal(), TestTCsGoal(), TestIMUGoal()]
        system_component_label = [
            ["FR", "FL", "BR", "BL"],
            ["FR", "TFR", "TBR", "FL", "TFL", "TBL"],
            ["TC1", "TC2", "TC3", "TC4", "TC5", "TC6", "TC7", "TC8"],
            ["Pixhawk"],
        ]

        if button is None:
            self.run_systems_test_button = RunSystemsTest(size_hint=(1, 1))
            self.run_systems_test_button.ids.run_test_button.bind(on_release=self.run_systems_test)
            self.run_systems_test_button.label = "RUN SYSTEMS TEST"
            self.run_systems_test_button.ids.run_test_button.disabled = False
            self.indicator_container.add_widget(self.run_systems_test_button)
        else:
            self.run_systems_test_button = button
            self.run_systems_test_button.bind(on_release=self.run_systems_test)
            self.run_systems_test_button.text = "RUN SYSTEMS TEST"

        self.result_value = {"CC": [], "TOF": [], "TC": [], "IMU": [], "POWER": []}

        self.running = False
        self.armed = False
        self.arm_service = rospy.ServiceProxy("/Hullbot/bridge/mav_srv/arming", CommandBool)

        self.popup = Popup()

        for index in range(len(system_labels)):
            mySystemInd = SystemIndicator(
                action_topic=system_test_action[index],
                label=system_labels[index],
                action_type=type(system_action_type[index]),
                action_goal=system_action_goal[index],
            )  # these objects are of type NodeIndicator
            mySystemInd.setComponentLabels(system_component_label[index])
            mySystemInd.setActionType(system_action_type[index])
            mySystemInd.setActionGoal(system_action_goal[index])
            mySystemInd.create_action_client()
            self.systems_list.append(mySystemInd)

    def colour_text(self, text, colour):
        """
        Wraps text with colour tag for kivy label
        """
        if colour == "r":
            colour_tag = "[color=ff3333]"
        elif colour == "g":
            colour_tag = "[color=33ff33]"
        elif colour == "y":
            colour_tag = "[color=ff8833]"
        else:
            colour_tag = "[color=ffffff]"

        colour_text = colour_tag + text + "[/color]"
        return colour_text

    def state_cb(self, state):
        """
        Subscriber to detect when the bot is disarmed.  If the bot is disarmed while the test is running
        the systems test is preempted.
        """
        self.armed = state.armed
        # Cancel systems test if bot is disarmed while test is running
        if not state.armed and self.running:
            self.running = False
            self.run_systems_test_button.set_label("RUN SYSTEMS TEST")
            # If it finished not successfully, show an error
            for system in self.systems_list:
                system.ac.cancel_all_goals()

    def as_feedback_cb(self, feedback):
        """
        Feedback from test action server can be accessed through this cb
        Current feedback is id of system component being tested eg. 2 if testing tc 2
        """
        # self.result_value.append(feedback.feedback)
        pass

    def check_bot_upright(self):
        """
        Check if bot is orientated correctly with cleaning cores upright.
        This ensures data logging of system tests remains consistent.
        """
        try:
            attitude = rospy.wait_for_message(
                "/Hullbot/bridge/mav_sensor/attitude", AttitudeTarget, timeout=5
            )
        except (rospy.ROSInterruptException, rospy.ROSException) as e:
            rospy.logerr("Waiting for AttitudeTarget message failed: %s" % (e,))
            return False

        current_eul = quat_to_eul_deg(attitude.orientation)
        roll_deg = current_eul.x

        roll_tolerance = 30  # degree tolerance from upright position (0 deg)
        if roll_deg > -roll_tolerance and roll_deg < roll_tolerance:
            return True
        else:
            return False

    def check_voltage(self, voltage, nominal_voltage):
        return voltage < nominal_voltage * 1.1 and voltage > nominal_voltage * 0.9

    @threaded
    def start_test(self, *args):
        """
        Executes systems test by sending action goals to TC, CC, TOFs, IMU, and power to test each system
        """
        self.popup.dismiss()
        if not self.check_bot_upright():
            popup_text = (
                "Bot must be positioned with cleaning cores facing upright before starting test."
            )
            show_dismissable_popup(
                title="CHECK BOT ORIENTATION", text=popup_text, size_x=600, size_y=300
            )
            return

        if not self.armed:
            popup_text = "Bot must be armed before starting test."
            show_dismissable_popup(title="ARM BOT", text=popup_text, size_x=400, size_y=300)
            return

        self.running = True
        self.run_systems_test_button.set_label("Stop Systems Test")
        self.result_value = {"CC": None, "TOF": None, "TC": None, "IMU": None, "POWER": None}

        # Rev up thruster cores
        if not self.running:  # Check test is still running before sending a goal
            self.end_systems_test()
            return
        tc_system = self.systems_list[self.system_idx["TC"]]
        try:
            tc_system.ac.wait_for_server()
            tc_system.ac.send_goal(tc_system.action_goal, feedback_cb=self.as_feedback_cb)
            tc_system.ac.wait_for_result()  # Wait for server to excute goal (ie. test system)
            result = tc_system.ac.get_result()
            self.result_value["TC"] = result
            self.set_system_status(tc_system, result)
        except:
            rospy.logerr("Failed to send goal to {} action server".format(tc_system.label))
            tc_system.dead()

        # Arm bot after successfully completing TC test as timeout will disarm bot

        try:
            rospy.wait_for_service("/Hullbot/bridge/mav_srv/arming", timeout=5)
        except:
            rospy.logwarn("/Hullbot/bridge/mav_srv/arming service not available to arm bot")

        try:
            self.arm_service(True)
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to arm bot after TC test: " + str(e))

        # Test cleaning cores and tofs simultaneously
        cc_system = self.systems_list[self.system_idx["CC"]]
        tof_system = self.systems_list[self.system_idx["TOF"]]
        if not self.running:  # Check test is still running before sending a goal
            self.end_systems_test()
            return
        # Send goal to CC test AS
        try:
            cc_system.ac.wait_for_server()
            cc_system.ac.send_goal(cc_system.action_goal, feedback_cb=self.as_feedback_cb)
        except:
            rospy.logerr("Failed to send goal to {} action server".format(cc_system.label))
            cc_system.dead()

        if not self.running:  # Check test is still running before sending a goal
            self.end_systems_test()
            return
        # Send goal to TOF test AS
        try:
            tof_system.ac.wait_for_server()
            tof_system.ac.send_goal(tof_system.action_goal, feedback_cb=self.as_feedback_cb)
        except:
            rospy.logerr("Failed to send goal to {} action server".format(tof_system.label))
            tof_system.dead()

        tof_system.ac.wait_for_result()  # Wait for server to excute goal (ie. test system)
        result = tof_system.ac.get_result()
        self.result_value["TOF"] = result
        self.set_system_status(tof_system, result)

        cc_system.ac.wait_for_result()  # Wait for server to excute goal (ie. test system)
        result = cc_system.ac.get_result()
        self.result_value["CC"] = result
        self.set_system_status(cc_system, result)

        # Test Pixhawk IMU
        imu_system = self.systems_list[self.system_idx["IMU"]]
        if not self.running:  # Check test is still running before sending a goal
            self.end_systems_test()
            return
        # Send goal to Pixhawk IMU test AS
        try:
            imu_system.ac.wait_for_server()
            imu_system.ac.send_goal(imu_system.action_goal, feedback_cb=self.as_feedback_cb)
        except:
            rospy.logerr("Failed to send goal to {} action server".format(imu_system.label))
            imu_system.dead()

        finished = imu_system.ac.wait_for_result(
            timeout=rospy.Duration(5)
        )  # Wait for server to excute goal with 5 second timeout
        result = TestIMUResult()
        if finished:
            result = imu_system.ac.get_result()
        else:
            rospy.logerr("Timeout from {} action server".format(imu_system.label))
            result.status = [TEST_STATUS_FAILURE]

        self.result_value["IMU"] = result
        self.set_system_status(imu_system, result)

        # Test power statistics
        # Take 5 readings of power stats
        voltage_LED = 0
        voltage_12V = 0
        voltage_5V = 0

        for i in range(5):
            power_msg = rospy.wait_for_message("/Hullbot/sensors/power_board_adc", PowerBoardADC)
            voltage_LED += power_msg.led_voltage_12v
            voltage_12V += power_msg.voltage_12v
            voltage_5V += power_msg.voltage_5v

        # Take average
        voltage_LED /= 5
        voltage_12V /= 5
        voltage_5V /= 5

        result = {
            "voltage_LED": (voltage_LED, self.check_voltage(voltage_LED, 12)),
            "voltage_12V": (voltage_12V, self.check_voltage(voltage_12V, 12)),
            "voltage_5V": (voltage_5V, self.check_voltage(voltage_5V, 5)),
        }

        if result["voltage_LED"][1] and result["voltage_12V"][1] and result["voltage_5V"][1]:
            result["status"] = TEST_STATUS_SUCCESS
        else:
            result["status"] = TEST_STATUS_FAILURE

        self.result_value["POWER"] = result

        self.end_systems_test()

    def set_system_status(self, system, result):
        """
        Set overall status of system.  The status is set as follows:
        SUCCESS - All componenets passed the test
        WARNING - If any component status is warning but no failures occured
        FAILURE - If any component failed the test
        """
        working_count = 0
        error_count = 0
        for i, status in enumerate(result.status):
            if status == TEST_STATUS_SUCCESS:
                working_count += 1
            elif status == TEST_STATUS_FAILURE:
                error_count += 1
                rospy.loginfo("{} {}: failure".format(system.label, i))
            else:
                rospy.logwarn("{} {}: warning".format(system.label, i))
        # Set status of system
        if working_count == len(result.status):  # All components of system working and passed test
            system.set_status(TEST_STATUS_SUCCESS)
        elif error_count > 0:  # Any component failed
            system.set_status(TEST_STATUS_FAILURE)
        else:  # Some components did not pass test but working
            system.set_status(TEST_STATUS_WARNING)

    def display_results(self):
        """
        Displays a breakdown of results from systems test in a popup window
        """
        results_text = "Systems Test Results"
        status = TEST_STATUS_SUCCESS
        for system in self.systems_list:
            results_text += "\n" + system.label
            if system.status == TEST_STATUS_WARNING:
                results_text += " WARNING"
                if (
                    status != TEST_STATUS_FAILURE
                ):  # Don't want to overwrite status with warning if something has failed
                    status = TEST_STATUS_WARNING
            elif system.status == TEST_STATUS_FAILURE:
                results_text += " FAILED"
                status = TEST_STATUS_FAILURE
            elif system.status == TEST_STATUS_SUCCESS:
                results_text += " PASSED"
            # Get system units
            test_units = ""
            if system.label == "CC":
                test_units = "A"
            elif system.label == "TC":
                test_units = "A"
            elif system.label == "TOF":
                test_units = "m"
            elif system.label == "IMU":
                test_units = "NA"

            # Write individual component status to popup
            for i, result in enumerate(self.result_value[system.label].value):
                status_text = "{} {}: {}{}".format(
                    system.label, i + 1, round(result, 3), test_units
                )
                if self.result_value[system.label].status[i] == TEST_STATUS_FAILURE:
                    status_text = self.colour_text(status_text, "r")
                elif self.result_value[system.label].status[i] == TEST_STATUS_SUCCESS:
                    status_text = self.colour_text(status_text, "g")
                else:
                    status_text = self.colour_text(status_text, "y")
                results_text += "\n" + status_text
            # Write IMU status message
            if system.label == "IMU":
                if system.status == TEST_STATUS_SUCCESS:
                    status_text = "Pixhawk IMU Working"
                    status_text = self.colour_text(status_text, "g")
                else:
                    status_text = "Check Pixhawk IMU"
                    status_text = self.colour_text(status_text, "r")
                results_text += "\n" + status_text

        # Add result for power tests
        if self.result_value["POWER"]["status"] == TEST_STATUS_SUCCESS:
            status_text = "POWER PASSED"
        else:
            status_text = "POWER FAILED"
        results_text += "\n" + status_text

        for key in self.result_value["POWER"]:
            if key != "status":
                status_text = "{}: {}V".format(key, round(self.result_value["POWER"][key][0], 3))

                if self.result_value["POWER"][key][1]:
                    status_text = self.colour_text(status_text, "g")
                else:
                    status_text = self.colour_text(status_text, "r")
                results_text += "\n" + status_text

        # Popup to display status of each component
        self.status_pub.publish(Int8(status))
        show_dismissable_popup(
            "Results", results_text, size_x=400, size_y=650, ok_button_fn=None, ok_button_text="Ok"
        )

    def end_systems_test(self):
        """
        Displays results of test and reset systems test button
        """
        rospy.loginfo("Completed test")
        try:
            self.display_results()
        except:
            show_dismissable_popup(
                title="Systest Feedback Failed",
                text="Try re-run system test",
                size_x=600,
                size_y=300,
            )
        self.running = False
        self.run_systems_test_button.set_label("RUN SYSTEMS TEST")

        # Disarm robot after systems test
        try:
            self.arm_service(False)
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to disarm bot after systems test: " + str(e))

    @threaded
    def run_systems_test(self, *args):
        """
        Opens popup to start systems test.  The test function callback is bound to the "Start" button.
        """
        # Check if running
        if self.running:
            rospy.loginfo("STOP TEST")
            self.running = False
            self.run_systems_test_button.set_label("RUN SYSTEMS TEST")
            # If it finished not successfully, show an error
            for system in self.systems_list:
                system.ac.cancel_all_goals()
        else:
            popup_text = "You are about to start a hardware systems test."
            popup_text += (
                "\nDuring this test the thruster cores and cleaning cores will be spun up."
            )
            popup_text += (
                "\nEnsure they are free of any obstructions before proceeding with the test."
            )
            self.popup = show_dismissable_popup(
                title="STARTING HARDWARE SYSTEMS TEST",
                text=popup_text,
                size_x=600,
                size_y=400,
                ok_button_fn=self.start_test,
                ok_button_text="Start",
            )

            # Generate dismissable popup if no rosbag is currently recording
            if self._check_active_bag() == False:
                popup_text = "You are about to start a hardware systems test."
                popup_text += "\nHowever, no rosbag is currently recording."
                colour_popup_text = self.colour_text(popup_text, "r")
                self.popup = show_dismissable_popup(
                    title="ROSBAG RECORDING CONFIRMATION",
                    text=colour_popup_text,
                    size_x=600,
                    size_y=400,
                )

    @staticmethod
    def _check_active_bag():
        """
        Checks rosbag directory for any active bag files using the current date
        """
        last_modified_time = 0
        last_modified_rosbag = None

        # Generate mission name from current date and boat name
        bag_dir = os.path.expanduser("~/Hullbot_bag_files/")
        today = datetime.date.today()
        date = today.strftime("%y%m%d")  # Format: 211209

        # Check if boat name param exists
        if not (rospy.has_param("/boat_name")):
            return False

        boat_name = rospy.get_param("/boat_name")
        mission_name = date + "_" + boat_name
        current_dir = os.path.join(bag_dir, mission_name)
        try:
            files = os.listdir(current_dir)
        except OSError:
            print("Bag directory not found")
            return False

        # Find last modified rosbag within mission name directory
        for current_file in files:
            current_rosbag = os.path.join(bag_dir, mission_name, current_file)
            current_file_stats = os.stat(current_rosbag)

            if current_file_stats.st_mtime > last_modified_time:
                last_modified_time = current_file_stats.st_mtime
                last_modified_rosbag = current_rosbag

        # Check active status of last modified rosbag
        if last_modified_rosbag is not None and last_modified_rosbag.endswith(".active"):
            return True
        return False


if __name__ == "__main__":
    rospy.init_node("systems_tester", anonymous=True)

    class WidgetApp(App):
        def build(self):
            return SystemTestPane()

    WidgetApp().run()
