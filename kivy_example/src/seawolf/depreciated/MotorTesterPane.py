import os

import dynamic_reconfigure.client
import kivy
import rospy
from BuoyancyTestPane import BuoyancyTestPane
from dismissable_popup import show_dismissable_popup
from hullbot_msgs.msg import VescStatus, VescStatusArray
from kivy.app import App
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.properties import StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.togglebutton import ToggleButton
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from std_msgs.msg import Bool, Int16MultiArray
from SystemTestPane import SystemTestPane

from seawolf.utils.ThreadingSupport import threaded

kivy.require("1.2.0")

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("motorTesterPane.kv"))


class LabelSwitch(BoxLayout):
    name = StringProperty()

    def __init__(self, **kwargs):
        super(LabelSwitch, self).__init__(**kwargs)


class RunMotorDetect(BoxLayout):
    def __init__(self, **kwargs):
        super(RunMotorDetect, self).__init__(**kwargs)


class RunMotorTest(BoxLayout):
    def __init__(self, **kwargs):
        super(RunMotorTest, self).__init__(**kwargs)


class LabelSliderLabel(BoxLayout):
    name = StringProperty()

    def __init__(self, **kwargs):
        super(LabelSliderLabel, self).__init__(**kwargs)


class MotorSelectorPane(BoxLayout):
    def __init__(self, **kwargs):
        super(MotorSelectorPane, self).__init__(**kwargs)


class MotorTesterPane(BoxLayout):
    def __init__(self, **kwargs):
        super(MotorTesterPane, self).__init__(**kwargs)
        self.arm_service = rospy.ServiceProxy("/Hullbot/bridge/mav_srv/arming", CommandBool)
        # This is the publisher for thruster commands
        self.tc_pub = rospy.Publisher("Hullbot/bridge/send_tc_cmd", Int16MultiArray, queue_size=5)
        # This is the publisher for toggle duty/rpm mode universally, will affect other nodes, True for duty, False for RPM
        self.cc_toggle_pub = rospy.Publisher("/Hullbot/aux/toggle_duty_rpm", Bool, queue_size=1)

        # This is the publisher for cleaning motor commands, accepts duty/rpm
        # New cleaning core control publisher
        self.cleaning_cores_command_pub = rospy.Publisher(
            "/hullbot/cleaning_cores/command", VescStatusArray, queue_size=5
        )  # May want to use rosparam for queue_size

        # Cleaning cores dynamic reconfigure client
        self.cleaning_cores_node_client = None

        self.last_duty_limit = None
        self.last_rpm_limit = None

        self.repub_time = 5.0
        self.last_commands = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

        self.repub_timer = rospy.Timer(rospy.Duration(self.repub_time), self.republish_commands)

        self.neutral_pwm = 1500  # The neutral TC pwm command
        self.max_pwm = 1700
        self.min_pwm = 1300
        self.pwm_step = 10  # Step size for TC slider

        self.default_duty_lim = 0.25  # Default CC duty limit
        self.default_rpm_lim = 1500  # Default CC RPM limit
        # Step size for CC duty limit slider and CC duty command slider
        self.default_duty_step = 0.05
        # Step size for CC RPM limit slider and CC RPM command slider
        self.default_rpm_step = 50

        # Initialize all TC command to neutral pwm
        self.tc_cmd = [self.neutral_pwm] * 8
        self.stop_tc_cmd = [self.neutral_pwm] * 8

        # This grid_seq defines the widget sequence of grid layout, 'e' means the corresponding grid cell is supposed to be empty
        self.grid_seq = [
            "CC2",
            "e",
            "e",
            "CC1",
            "TC2",
            "e",
            "e",
            "TC1",
            "e",
            "TC6",
            "TC5",
            "e",
            "e",
            "TC8",
            "TC7",
            "e",
            "TC4",
            "e",
            "e",
            "TC3",
            "CC4",
            "e",
            "e",
            "CC3",
        ]
        # Initialize a list of TC toggle buttons
        self.tc_tb_list = [None] * 8
        # Initialize a list of CC toggle buttons
        self.cc_tb_list = [None] * 4
        self.motor_selector = MotorSelectorPane()
        for cell in self.grid_seq:
            if cell[0:2] == "TC":
                """If the cell is supposed to be TC, add it to the TC toggle button list at corresponding index"""
                tb_temp = ToggleButton(text=cell)
                self.tc_tb_list[int(cell[2]) - 1] = tb_temp
            elif cell[0:2] == "CC":
                """If the cell is supposed to be CC, add it to the CC toggle button list at corresponding index"""
                tb_temp = ToggleButton(text=cell)
                self.cc_tb_list[int(cell[2]) - 1] = tb_temp
            else:
                """Add an empty label to remaining cells"""
                tb_temp = Label(text="")
            self.motor_selector.ids.motor_grid.add_widget(tb_temp)  # Add the widget to the grid

        # Bind CC toggle button callback functions
        self.cc_tb_list[0].bind(on_release=self.cc0ToggleCallback)
        self.cc_tb_list[1].bind(on_release=self.cc1ToggleCallback)
        self.cc_tb_list[2].bind(on_release=self.cc2ToggleCallback)
        self.cc_tb_list[3].bind(on_release=self.cc3ToggleCallback)

        # Disable TC and CC toggle buttons
        for i in range(0, 8):
            self.tc_tb_list[i].disabled = True
        for i in range(0, 4):
            self.cc_tb_list[i].disabled = True

        self.system_test_pane = SystemTestPane(size_hint=(1, 0.1))
        self.ids.test_panel.add_widget(self.system_test_pane)
        self.buoyancy_test_pane = BuoyancyTestPane(size_hint=(1, 0.1))
        self.ids.test_panel.add_widget(self.buoyancy_test_pane)

        # Add a switch for TCs which activates all TC toggle buttons
        self.TC_switch = LabelSwitch(name="TC switch", size_hint=(1, 0.1))
        self.TC_switch.ids.switch.bind(active=self.TC_switch_callback)
        self.ids.test_panel.add_widget(self.TC_switch)
        # Add a switch for CCs which activates all CC toggle buttons
        self.CC_switch = LabelSwitch(name="CC switch", size_hint=(1, 0.1))
        self.CC_switch.ids.switch.bind(active=self.CC_switch_callback)
        self.ids.test_panel.add_widget(self.CC_switch)

        # Add Buttons for run TC, run TC rinse, CC test and detect TC motor direction
        self.run_test_btns = RunMotorTest(size_hint=(1, 0.2))
        self.run_test_btns.ids.runTC_btn.bind(on_release=self.tc_test_btn_cb)
        self.run_test_btns.ids.runTC_btn.disabled = True
        self.run_test_btns.ids.runTCR_btn.bind(on_release=self.tc_rinse_btn_cb)
        self.run_test_btns.ids.runTCR_btn.disabled = True
        self.run_test_btns.ids.runCC_btn.bind(on_release=self.cc_test_btn_cb)
        self.run_test_btns.ids.runCC_btn.disabled = True
        self.run_test_btns.ids.detectTC_btn.bind(on_release=self.run_motor_detect)
        self.run_test_btns.ids.detectTC_btn.disabled = True

        self.ids.test_panel.add_widget(self.run_test_btns)

        self.popup = Popup()

        # Add toggle button grid to main widget
        self.ids.test_panel.add_widget(self.motor_selector)

        # Add TC slider
        self.tc_slider = LabelSliderLabel(name="TC", size_hint=(1, 0.15))
        self.tc_slider.ids.slider.min = self.min_pwm
        self.tc_slider.ids.slider.max = self.max_pwm
        self.tc_slider.ids.slider.value = self.neutral_pwm
        self.tc_slider.ids.slider.step = self.pwm_step
        self.ids.test_panel.add_widget(self.tc_slider)

        # Add a switch which toggles RPM/Duty mode
        self.RPM_switch = LabelSwitch(name="RPM mode", size_hint=(1, 0.1))
        self.RPM_switch.ids.switch.bind(active=self.toggle_rpm)
        self.ids.test_panel.add_widget(self.RPM_switch)

        # Add a slider which controls the CC duty and RPM limit
        self.limit_slider = LabelSliderLabel(name="Duty limit", size_hint=(1, 0.15))
        self.limit_slider.ids.slider.max = 0.5
        self.limit_slider.ids.slider.min = 0.1
        self.limit_slider.ids.slider.step = self.default_duty_step
        self.limit_slider.ids.slider.value = self.default_duty_lim
        self.limit_slider.ids.slider.bind(value=self.limit_slider_cb)
        self.ids.test_panel.add_widget(self.limit_slider)

        # Add CC command slider
        self.cc_slider = LabelSliderLabel(name="Duty", size_hint=(1, 0.15))
        self.cc_slider.ids.slider.value = 0
        self.cc_slider.ids.slider.max = self.limit_slider.ids.slider.value
        self.cc_slider.ids.slider.min = -self.limit_slider.ids.slider.value
        self.cc_slider.ids.slider.step = self.default_duty_step
        # Bind CC slider callback function, this callback will be triggered everytime CC slider value changes
        self.cc_slider.ids.slider.bind(value=self.ccSliderCallback)
        self.ids.test_panel.add_widget(self.cc_slider)

        # Start thruster command clock event at 10Hz
        self.TC_event = Clock.schedule_interval(self.spinThruster, 0.1)
        # Turn off the event, thruster command should only be clocked when switched on
        self.TC_event.cancel()

        # TC motor test
        self.tcTest_tb_idx = 0  # This index indicates which TC is being tested
        # This list stores all pwm commands that will be sweeped during test
        self.pwm_test_seq = []
        pwm_t = self.neutral_pwm

        """ make a list of pwm command for motor test
            this list will be iterated through at TC test event
            1500 -> ....hold 1600.... -> 1500
        """
        self.tc_test_hold_time = 2  # thruster spin duration in seconds
        self.tc_test_gap_time = 4  # gap between consecutive thruster spins in seconds
        self.tc_test_hold_pwm = 1600  # hold pwm at 1600
        # clock schedule interval (s) - test pwms must be scheduled at this period.
        self.tc_test_period = 0.1
        while pwm_t < self.tc_test_hold_pwm:  # increment pwm from 1500 to 1600
            pwm_t = pwm_t + self.pwm_step
            self.pwm_test_seq.append(pwm_t)
        # append 1600 according to hold time
        for i in range(0, int(self.tc_test_hold_time / self.tc_test_period)):
            self.pwm_test_seq.append(self.tc_test_hold_pwm)
        # append 1500 according to gap time between motors
        for i in range(0, int(self.tc_test_gap_time / self.tc_test_period)):
            self.pwm_test_seq.append(self.neutral_pwm)

        # tcTest flag indicates if the TC test is currently running
        self.tcTest_flag = False
        # this is the universal index of TC test pwm command list, this is used as an iterator to loop through pwm command list,
        self.tcTest_pwm_idx = 0
        # it will be accessed and incremented in TC test event
        # Create the TC test event object, this event will only be triggered/cancelled by TC test button
        self.TC_test_event = Clock.schedule_interval(self.run_TC_test, self.tc_test_period)
        self.TC_test_event.cancel()

        # #TC Rinse Sequence
        self.tcRinse_tb_idx = 0  # This index indicates which TCs are being rinsed
        self.pwm_rinse_seq = []  # This list stores pwm commands for the TC rinse sequence
        pwm_r = self.neutral_pwm

        """ make a list of pwm commands for TC rinse
            this list will be iterated through at TC rinse event
            1500 -> ....hold 1600.... -> 1500 -> ....hold 1400.... -> 1500
        """

        self.tc_rinse_hold_time = 4  # thruster rinse duration in seconds
        # gap between consecutive thruster rinses in seconds
        self.tc_rinse_gap_time = 0.1
        self.tc_rinse_hold_pwm_cw = 1600  # hold pwm at 1600 clockwise
        self.tc_rinse_hold_pwm_ccw = 1400  # hold pwm at 1400 counter clockwise
        # clock schedule interval (s) - rinse pwms must be scheduled at this period.
        self.tc_rinse_period = 0.1
        while pwm_r < self.tc_rinse_hold_pwm_cw:  # increment pwm from 1500 to 1600
            pwm_r = pwm_r + self.pwm_step
            self.pwm_rinse_seq.append(pwm_r)
        # append 1600 according to hold time
        for i in range(0, int(self.tc_rinse_hold_time / self.tc_rinse_period)):
            self.pwm_rinse_seq.append(self.tc_rinse_hold_pwm_cw)
        # append 1500 according to gap time between motors
        for i in range(0, int(self.tc_rinse_gap_time / self.tc_rinse_period)):
            self.pwm_rinse_seq.append(self.neutral_pwm)
            pwm_r = self.neutral_pwm
        while pwm_r > self.tc_rinse_hold_pwm_ccw:  # decrement pwm from 1500 to 1400
            pwm_r = pwm_r - self.pwm_step
            self.pwm_rinse_seq.append(pwm_r)
        # append 1400 according to hold time
        for i in range(0, int(self.tc_rinse_hold_time / self.tc_rinse_period)):
            self.pwm_rinse_seq.append(self.tc_rinse_hold_pwm_ccw)
        # append 1500 according to gap time between motors
        for i in range(0, int(self.tc_rinse_gap_time / self.tc_rinse_period)):
            self.pwm_rinse_seq.append(self.neutral_pwm)

        # tcRinse flag indicates if the TC rinse is currently running
        self.tcRinse_flag = False
        # universal index of TC rinse pwm command list, used as an iterator to loop through pwm command list,
        self.tcRinse_pwm_idx = 0
        # it will be accessed and incremented in TC rinse event
        # Create the TC rinse event object, this event will only be triggered/cancelled by TC rinse button
        self.TC_rinse_event = Clock.schedule_interval(self.run_TC_rinse, self.tc_rinse_period)
        self.TC_rinse_event.cancel()

        # CC test sequence
        self.ccTest_tb_idx = 0
        self.duty_test_seq = []
        duty_t = 0.00

        """make a list of duty commands
            this list will be iterated through at CC test event
            0 -> 0.25 -> -0.25 -> 0
        """
        while (
            round(duty_t, 2) < self.cc_slider.ids.slider.max
        ):  # append duty from 0 to 0.25 to list
            duty_t = round(duty_t + self.cc_slider.ids.slider.step, 2)
            self.duty_test_seq.append(duty_t)
        while duty_t > self.cc_slider.ids.slider.min:  # append duty from 0.25 to -0.25 to list
            duty_t = round(duty_t - self.cc_slider.ids.slider.step, 2)
            self.duty_test_seq.append(duty_t)
        while duty_t < 0:  # append duty from -0.25 to 0 to list
            duty_t = round(duty_t + self.cc_slider.ids.slider.step, 2)
            self.duty_test_seq.append(duty_t)
        # ccTest flag indicates if the CC test is currently running
        self.ccTest_flag = False
        self.ccTest_duty_idx = 0

        self.CC_test_event = Clock.schedule_interval(self.run_CC_test, 0.2)
        self.CC_test_event.cancel()

    @threaded
    def get_cc_node_client(self):
        """
        Initialiase the dynamic_reconfigure client to update max duty and max rpm config
        """
        try:
            self.cleaning_cores_node_client = dynamic_reconfigure.client.Client(
                "cleaning_cores_node", timeout=5, config_callback=self._cc_client_config_cb
            )

            # Reset limit to default value
            if self.RPM_switch.ids.switch.active:
                self.limit_slider.ids.slider.value = self.default_rpm_lim
            else:
                self.limit_slider.ids.slider.value = self.default_duty_lim

        except rospy.ROSException as e:
            print(
                "Could not establish client with 'cleaning_cores_node'.  Ensure cleaning cores node is running.\n{}".format(
                    e
                )
            )
            self.cleaning_cores_node_client = None

    def _cc_client_config_cb(self, config, *args):
        # Get the repub time
        self.repub_time = config["command_timeout"]

    def set_cc_duty_limit(self, lim):
        """
        Update cleaning core duty cycle limit through dynamic reconfigure client
        """
        if self.cleaning_cores_node_client:
            try:
                # Only publish duty limit if it has changed. Don't spam the dynamic reconfigure
                if self.last_duty_limit is None:
                    self.last_duty_limit = lim
                    self.cleaning_cores_node_client.update_configuration({"max_duty_cycle": lim})
                elif self.last_duty_limit != lim:
                    self.cleaning_cores_node_client.update_configuration({"max_duty_cycle": lim})
                    self.last_duty_limit = lim
            except dynamic_reconfigure.DynamicReconfigureCallbackException as e:
                print("Cleaning cores dynamic_reconfigure service not found: {}".format(e))
                self.cleaning_cores_node_client = None
        else:
            print("Cannot update limit. Cleaning cores dynamic_reconfigure client not up.")
            print("Attempting to establish client.")
            self.get_cc_node_client()

    def set_cc_rpm_limit(self, lim):
        """
        Update cleaning core rpm limit through dynamic reconfigure client
        """
        if self.cleaning_cores_node_client:
            try:
                # Only publish rpm limit if it has changed. Don't spam the dynamic reconfigure
                if self.last_rpm_limit is None:
                    self.last_rpm_limit = lim
                    self.cleaning_cores_node_client.update_configuration({"max_rpm": lim})
                elif self.last_rpm_limit != lim:
                    self.cleaning_cores_node_client.update_configuration({"max_rpm": lim})
                    self.last_rpm_limit = lim
            except dynamic_reconfigure.DynamicReconfigureCallbackException as e:
                print("Cleaning cores dynamic_reconfigure service not found: {}".format(e))
                self.cleaning_cores_node_client = None
        else:
            print("Cannot update limit. Cleaning cores dynamic_reconfigure client not up.")
            print("Attempting to establish client.")
            self.get_cc_node_client()

    def republish_commands(self, _):
        """
        Republish the last received command, if its not 0
        """
        for cleaning_core_id in range(1, 5):
            if self.last_commands[cleaning_core_id] == 0.0:
                continue
            self.publish_cc_command(self.last_commands[cleaning_core_id], cleaning_core_id - 1)

    def TC_switch_callback(self, *args):
        if self.TC_switch.ids.switch.active:
            """if TC switch is switched on, ARM the Bot"""
            for i in range(0, 8):
                # Enable all TC toggle buttons
                self.tc_tb_list[i].disabled = False
            # Enable run TC test and rinse all TCs buttons
            self.run_test_btns.ids.runTC_btn.disabled = False
            self.run_test_btns.ids.runTCR_btn.disabled = False
            self.run_test_btns.ids.detectTC_btn.disabled = False
            try:
                self.arm_service(True)
            except:
                print("Arm Failed")
            self.TC_event()  # Clock on the TC event, start to publish TC command
        else:
            # if TC test is operating
            if self.tcTest_flag:
                self.trigger_event()  # trigger the clock events back
                self.tcTest_tb_idx = 0  # reset test toggle button index
                self.tcTest_pwm_idx = 0  # reset test pwm index
                self.reset_tc_state()  # reset all TC states
                self.tcTest_flag = False  # tc Test flag off
            # if TC rinse is operating
            elif self.tcRinse_flag:
                self.trigger_event()  # trigger the clock events back
                self.tcRinse_tb_idx = 0  # reset test toggle button index
                self.tcRinse_pwm_idx = 0  # reset test pwm index
                self.reset_tc_state()  # reset all TC states
                self.tcRinse_flag = False  # tc Test flag off

            # Set TC command back to neutral
            self.tc_slider.ids.slider.value = self.neutral_pwm
            self.TC_event.cancel()  # Clock off the TC event
            for i in range(0, 8):
                # Make sure the TC toggle buttons are popped up
                self.tc_tb_list[i].state = "normal"
                # Disable all TC toggle buttons
                self.tc_tb_list[i].disabled = True
            # Disable run TC test and rinse TCs buttons
            self.run_test_btns.ids.runTC_btn.disabled = True
            self.run_test_btns.ids.runTCR_btn.disabled = True
            self.run_test_btns.ids.detectTC_btn.disabled = True
            self.tc_pub.publish(Int16MultiArray(data=self.stop_tc_cmd))

    def CC_switch_callback(self, *args):
        if self.CC_switch.ids.switch.active:
            for i in range(0, 4):
                # Enable all CC buttons
                self.cc_tb_list[i].disabled = False
            # Enable run CC test button
            self.run_test_btns.ids.runCC_btn.disabled = False
        else:
            if self.ccTest_flag:
                self.CC_test_event.cancel()
                self.ccTest_tb_idx = 0
                self.ccTest_duty_idx = 0
                self.cc_slider.ids.slider.value = 0

            for i in range(0, 4):
                # Disable all CC buttons
                self.cc_tb_list[i].state = "normal"
                self.cc_tb_list[i].disabled = True
            # Disable run CC test button
            self.run_test_btns.ids.runCC_btn.disabled = True
            self.cc_slider.ids.slider.value = 0

    """ ------------------------------------------------------------------
        Functions for Test TC button
    """

    def tc_test_btn_cb(self, *args):
        if not self.tcTest_flag:  # if motor test is not running
            self.reset_tc_state()
            self.TC_event.cancel()  # cancel normal TC event and start TC test event
            self.TC_test_event()
            self.tcTest_flag = True  # trigger motor test flag, test is started
            for i in range(0, len(self.tc_tb_list)):
                # Disable all TC buttons
                self.tc_tb_list[i].disabled = True
            self.run_test_btns.ids.runTCR_btn.disabled = True  # Disable TC rinse button
            self.run_test_btns.ids.detectTC_btn.disabled = True
        else:  # if motor test is running but test button is pressed again, ongoing test will terminate
            self.trigger_event()  # trigger event back to normal
            # reset TC toggle button index, so next test will start from beginning
            self.tcTest_tb_idx = 0
            self.tcTest_pwm_idx = 0  # reset TC pwm command index
            self.reset_tc_state()  # reset all TC states
            self.tcTest_flag = False  # test is terminated, reset TC test flag
            for i in range(0, len(self.tc_tb_list)):
                self.tc_tb_list[i].disabled = False  # Enable all TC buttons
            self.run_test_btns.ids.runTCR_btn.disabled = False  # Enable TC rinse button
            self.run_test_btns.ids.detectTC_btn.disabled = False

    def reset_tc_state(self):
        """This method resets all TC commands and pops up all TC toggle buttons"""
        for i in range(0, len(self.tc_tb_list)):
            # reset TC commands to neutral
            self.tc_cmd[i] = self.neutral_pwm
            # pop up all TC toggle buttons
            self.tc_tb_list[i].state = "normal"
        self.tc_slider.ids.slider.value = self.neutral_pwm  # reset slider value

    def trigger_event(self):
        """Turns off motor test event, turns on normal TC command event"""
        self.TC_test_event.cancel()  # Turn off motor test event
        self.TC_rinse_event.cancel()  # Turn off motor rinse event
        self.TC_event()  # Turn on normal TC command event

    def run_TC_test(self, dt):
        """This clock event loops through all TCs, and sweeps through all PWM commands for each TC"""
        if self.tcTest_pwm_idx == len(self.pwm_test_seq):  # if reaches end of pwm command list
            self.tcTest_pwm_idx = 0  # reset pwm command index
            # pop up current toggle button
            self.tc_tb_list[self.tcTest_tb_idx].state = "normal"
            self.tcTest_tb_idx = self.tcTest_tb_idx + 1  # increment toggle button index
        # if reaches end of toggle button list
        if self.tcTest_tb_idx == len(self.tc_tb_list):
            self.tcTest_tb_idx = 0  # reset TC index
            self.tcTest_flag = False  # test is about to end, reset TC test flag
            self.trigger_event()  # trigger event back
            for i in range(0, len(self.tc_tb_list)):
                self.tc_tb_list[i].disabled = False  # Enable all TC buttons
            self.run_test_btns.ids.runTCR_btn.disabled = False  # Enable TC rinse button
            return  # terminate current clock event

        # press down the current toggle button
        self.tc_tb_list[self.tcTest_tb_idx].state = "down"
        # update TC command list
        self.tc_cmd[self.tcTest_tb_idx] = self.pwm_test_seq[self.tcTest_pwm_idx]
        # update slider value
        self.tc_slider.ids.slider.value = self.pwm_test_seq[self.tcTest_pwm_idx]
        # publish TC command list
        self.tc_pub.publish(Int16MultiArray(data=self.tc_cmd))
        self.tcTest_pwm_idx = self.tcTest_pwm_idx + 1  # increment pwm command list

    """ End of TC test functions
    --------------------------------------------------------------------------
        TC rinse functions
    """

    def tc_rinse_btn_cb(self, *args):
        if not self.tcRinse_flag:  # if motor test is not running
            self.reset_tc_state()
            self.TC_event.cancel()  # cancel normal TC event and start TC rinse event
            self.TC_rinse_event()
            self.tcRinse_flag = True  # trigger motor rinse flag, test is started
            for i in range(0, len(self.tc_tb_list)):
                # Disable all TC buttons
                self.tc_tb_list[i].disabled = True
            self.run_test_btns.ids.runTC_btn.disabled = True  # Disable TC test button
            self.run_test_btns.ids.detectTC_btn.disabled = True
        else:  # if motor test is running but test button is pressed again, ongoing test will terminate
            self.trigger_event()  # trigger event back to normal
            # reset TC toggle button index, so next test will start from beginning
            self.tcRinse_tb_idx = 0
            self.tcRinse_pwm_idx = 0  # reset TC pwm command index
            self.reset_tc_state()  # reset all TC states
            self.tcRinse_flag = False  # test is terminated, reset TC test flag
            for i in range(0, len(self.tc_tb_list)):
                self.tc_tb_list[i].disabled = False  # Enable all TC buttons
            self.run_test_btns.ids.runTC_btn.disabled = False  # Enable TC test button
            self.run_test_btns.ids.detectTC_btn.disabled = False

    def run_TC_rinse(self, dt):
        """This clock event loops through all TCs, and sweeps through all PWM commands for each TC"""
        if self.tcRinse_pwm_idx == len(self.pwm_rinse_seq):  # if reaches end of pwm command list
            self.tcRinse_pwm_idx = 0  # reset pwm command index
            # if tcRinse_tb_idx < 4:
            # pop up current toggle button
            self.tc_tb_list[self.tcRinse_tb_idx].state = "normal"
            self.tcRinse_tb_idx = self.tcRinse_tb_idx + 1  # increment toggle button index
        # if reaches end of toggle button list
        if self.tcRinse_tb_idx == len(self.tc_tb_list):
            self.tcRinse_tb_idx = 0  # reset TC index
            self.tcRinse_flag = False  # test is about to end, reset TC rinse flag
            self.trigger_event()  # trigger event back
            for i in range(0, len(self.tc_tb_list)):
                self.tc_tb_list[i].disabled = False  # Enable all TC buttons
            self.run_test_btns.ids.runTC_btn.disabled = False  # Enable TC test button
            return  # terminate current clock event

        # press down the current toggle button
        self.tc_tb_list[self.tcRinse_tb_idx].state = "down"
        # update TC command list
        self.tc_cmd[self.tcRinse_tb_idx] = self.pwm_rinse_seq[self.tcRinse_pwm_idx]
        # update slider value
        self.tc_slider.ids.slider.value = self.pwm_rinse_seq[self.tcRinse_pwm_idx]
        # publish TC command list
        self.tc_pub.publish(Int16MultiArray(data=self.tc_cmd))
        self.tcRinse_pwm_idx = self.tcRinse_pwm_idx + 1  # increment pwm command list

    """ End of TC rinse functions
    --------------------------------------------------------------------------
        CC test functions
    """

    def cc_test_btn_cb(self, *args):
        if not self.ccTest_flag:  # if motor test is not running
            self.cc_slider.ids.slider.value = 0
            self.CC_test_event()
            self.ccTest_flag = True  # trigger motor test flag, test is started
            for i in range(0, 4):
                self.cc_tb_list[i].disabled = True
        else:  # if motor test is running but test button is pressed again, ongoing test will terminate
            self.CC_test_event.cancel()
            # reset TC toggle button index, so next test will start from beginning
            self.ccTest_tb_idx = 0
            self.ccTest_duty_idx = 0  # reset TC pwm command index
            self.cc_slider.ids.slider.value = 0
            self.ccTest_flag = False  # test is terminated, reset TC test flag
            for i in range(0, 4):
                self.cc_tb_list[i].state = "normal"
                self.cc_tb_list[i].disabled = False
        pass

    def run_CC_test(self, dt):
        if self.RPM_switch.ids.switch.active:  # If RPM mode is on, switch off RPM mode
            self.RPM_switch.ids.switch.active = False
        # if reached end of duty list
        if self.ccTest_duty_idx == len(self.duty_test_seq):
            self.ccTest_duty_idx = 0
            self.cc_tb_list[self.ccTest_tb_idx].state = "normal"
            # increment toggle button index to goto next CC
            self.ccTest_tb_idx = self.ccTest_tb_idx + 1
        # if reaches end of toggle button list
        if self.ccTest_tb_idx == len(self.cc_tb_list):
            self.ccTest_tb_idx = 0  # reset CC index
            self.ccTest_flag = False  # test is about to end, reset CC test flag
            self.CC_test_event.cancel()  # cancel CC test event
            for i in range(0, 4):
                # Enable all CC buttons
                self.cc_tb_list[i].disabled = False
            return
        # press down the current toggle button
        self.cc_tb_list[self.ccTest_tb_idx].state = "down"
        # update slider value
        self.cc_slider.ids.slider.value = self.duty_test_seq[self.ccTest_duty_idx]
        self.ccTest_duty_idx = self.ccTest_duty_idx + 1  # increment duty command list

        pass

    """
        End of CC test functions
    ----------------------------------------------------------------------------
        Motor detect functions
    """

    @threaded
    def run_motor_detect(self, *args):
        popup_text = "YOU ARE ABOUT TO RECALIBRATE THE THRUSTER CORES\n\n"
        popup_text += "ARE YOU SURE WANT TO DO THIS?\n\n"
        popup_text += "The bot must be in the water and in an upright position to run this test.\n"
        popup_text += "Confirm proper calibration by swimming the bot after test."

        self.popup = show_dismissable_popup(
            title="WARNING",
            text=popup_text,
            size_x=600,
            size_y=400,
            ok_button_fn=self.start_detect_motor_test,
            ok_button_text="Start",
        )

    def start_detect_motor_test(self, *args):
        self.popup.dismiss()

        motor_detect_service = rospy.ServiceProxy(
            "/Hullbot/bridge/mav_srv/motor_detect", CommandBool
        )
        popup_text = ""
        try:
            response = motor_detect_service.call(CommandBoolRequest(True))
            popup_text = "Sent request to calibrate motors:\n{}".format(response)
        except Exception as e:
            popup_text = "Exception when calling motor_detect service\n({})".format(e)

        # Toggle TC switch off
        self.TC_switch.ids.switch.active = False
        self.TC_switch_callback()

        show_dismissable_popup(
            title="Detect Motor Spin Direction", text=popup_text, size_x=500, size_y=300
        )

    """
    ----------------------------------------------------------------------------
    End of Motor detect functions"""

    def limit_slider_cb(self, *args):
        """This callback is triggered everytime the limit slider value changes"""
        if self.RPM_switch.ids.switch.active:
            self.set_cc_rpm_limit(self.limit_slider.ids.slider.value)
        else:
            self.set_cc_duty_limit(self.limit_slider.ids.slider.value)
        # Update the CC command slider limits after limit change
        self.update_cc_slider_limit()

    def update_cc_slider_limit(self):
        self.cc_slider.ids.slider.max = self.limit_slider.ids.slider.value
        self.cc_slider.ids.slider.min = -self.limit_slider.ids.slider.value

    def toggle_rpm(self, *args):
        """This callback toggles CC limit slider and CC command slider to RPM/Duty Mode"""
        if self.RPM_switch.ids.switch.active:  # If switched on, toggle to RPM mode
            # Publish to cc toggle topic, False for RPM, True for Duty
            self.cc_toggle_pub.publish(Bool(data=False))
            self.limit_slider.name = "RPM limit"  # Change slider labels
            self.cc_slider.name = "RPM"
            self.limit_slider.ids.slider.min = 1000  # Change CC limit slider properties
            self.limit_slider.ids.slider.max = 4000
            self.limit_slider.ids.slider.step = self.default_rpm_step
            self.limit_slider.ids.slider.value = self.default_rpm_lim
            # Set duty limit back to default to prevent RPM/Duty limit set together
            self.set_cc_duty_limit(self.default_duty_lim)
            self.cc_slider.ids.slider.value = 0  # Change CC command slider properties
            self.update_cc_slider_limit()  # update CC the slider limits
            self.cc_slider.ids.slider.step = self.default_rpm_step
        else:
            self.cc_toggle_pub.publish(Bool(data=True))
            self.cc_slider.ids.slider.value = 0
            self.limit_slider.name = "Duty limit"
            self.cc_slider.name = "Duty"
            self.limit_slider.ids.slider.min = 0.1
            self.limit_slider.ids.slider.max = 0.5
            self.limit_slider.ids.slider.step = self.default_duty_step
            self.limit_slider.ids.slider.value = self.default_duty_lim
            # Set rpm limit back to default to prevent RPM/Duty limit set together
            self.set_cc_rpm_limit(self.default_rpm_lim)
            self.update_cc_slider_limit()
            self.cc_slider.ids.slider.step = self.default_duty_step

    """These CC callbacks spins the corresponding cleaning motor
        if the associated toggle button is pressed down,
        if the button is popped up, the motor will be stopped
    """

    def cc0ToggleCallback(self, *args):
        self.spinCleaner(ccid=0)

    def cc1ToggleCallback(self, *args):
        self.spinCleaner(ccid=1)

    def cc2ToggleCallback(self, *args):
        self.spinCleaner(ccid=2)

    def cc3ToggleCallback(self, *args):
        self.spinCleaner(ccid=3)

    def ccSliderCallback(self, *args):
        """This is the CC command slider callback, it attempts to spin all CC motors when slider value changes"""
        for i in range(0, len(self.cc_tb_list)):
            # CC toggle button state will be checked before spin, if button pressed, then spin, if not, then publish 0
            self.spinCleaner(ccid=i)

    def spinThruster(self, dt):
        """This is the clock event for TCs"""
        for i in range(0, 8):
            """Check if the toggle button is pressed, if pressed, update the slider value to corresponding index
            if no, set the command to neutral"""
            if self.tc_tb_list[i].state == "down":
                self.tc_cmd[i] = self.tc_slider.ids.slider.value
            else:
                self.tc_cmd[i] = self.neutral_pwm
        # publish the command
        self.tc_pub.publish(Int16MultiArray(data=self.tc_cmd))

    def publish_cc_command(self, val, ccid):
        """Spin given cleaning core by publishing cc command"""
        vsa = VescStatusArray()
        s = VescStatus()
        s.vesc_id = ccid + 1

        self.last_commands[s.vesc_id] = val
        if abs(val) <= 1.0:
            s.duty_cycle = val
        else:
            s.rpm = val
        vsa.status.append(s)
        self.cleaning_cores_command_pub.publish(vsa)

    def spinCleaner(self, ccid):
        """Check associated cc toggle button state before sending cc command"""
        if self.cc_tb_list[ccid].state == "down":
            val = self.cc_slider.ids.slider.value
        else:
            val = 0.0

        self.publish_cc_command(val, ccid)


if __name__ == "__main__":
    rospy.init_node("motor_tester", anonymous=True)

    class WidgetApp(App):
        def build(self):
            return MotorTesterPane()

    WidgetApp().run()
