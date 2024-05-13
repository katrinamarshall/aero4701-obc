import json
import os
import sys

import kivy
import rospy

kivy.require("1.2.0")
import subprocess

import pubs
import subs
from BatteryDisplay import BatteryVoltageWarningPopup
from diagnostic_msgs.msg import DiagnosticArray
from hullbot_msgs.msg import PowerBoardADC, VescStatusArray
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import (
    ListProperty,
    NumericProperty,
    ObjectProperty,
    StringProperty,
)
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.popup import Popup
from mavros_msgs.msg import VFR_HUD, RCOut, State
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int8, Int16, String

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("topicviewpane.kv"))


CURRENT_THRESHOLD = 50.0
USABLE_AH = 7.2

SLAM_SYSTEM_NOT_READY = -1
SLAM_NO_IMAGES_YET = 0
SLAM_NOT_INITIALISED = 1
SLAM_OK = 2
SLAM_RECENTLY_LOST = 3
SLAM_LOST = 4


class TopicViewPane(BoxLayout):
    armed = StringProperty()
    mode = StringProperty()
    connected = StringProperty()
    bat_voltage = StringProperty()
    bat_current = StringProperty()
    cleaning_port_2_duty = StringProperty()
    cleaning_port_4_duty = StringProperty()
    cleaning_star_1_duty = StringProperty()
    cleaning_star_3_duty = StringProperty()
    cleaning_port_2_rpm = StringProperty()
    cleaning_port_4_rpm = StringProperty()
    cleaning_star_1_rpm = StringProperty()
    cleaning_star_3_rpm = StringProperty()
    cc_curr_2 = StringProperty()
    cc_curr_4 = StringProperty()
    cc_curr_1 = StringProperty()
    cc_curr_3 = StringProperty()
    depth = StringProperty()
    bot_state = StringProperty("None")
    tc_pwm_1 = StringProperty()
    tc_pwm_2 = StringProperty()
    tc_pwm_3 = StringProperty()
    tc_pwm_4 = StringProperty()
    tc_pwm_5 = StringProperty()
    tc_pwm_6 = StringProperty()
    tc_pwm_7 = StringProperty()
    tc_pwm_8 = StringProperty()
    tc_curr_1 = StringProperty()
    tc_curr_2 = StringProperty()
    tc_curr_3 = StringProperty()
    tc_curr_4 = StringProperty()
    tc_curr_5 = StringProperty()
    tc_curr_6 = StringProperty()
    tc_curr_7 = StringProperty()
    tc_curr_8 = StringProperty()
    control_mode = StringProperty()
    tc_boost = StringProperty("Inactive")
    arm_status_led_label = ObjectProperty()
    conn_status_led_label = ObjectProperty()
    voltage_led_label = ObjectProperty()
    current_led_label = ObjectProperty()
    tc_boost_led_label = ObjectProperty()
    cpu_load_str = StringProperty()
    rovio_happiness = StringProperty()
    slam_atlas_map_count = StringProperty()
    rovio_memory_usage = StringProperty()
    is_underwater = StringProperty("Unknown")
    underwater_box_col = ListProperty([0.5, 0.5, 0.5, 1])
    batt_percent = StringProperty()
    batt_time = StringProperty()
    ping = StringProperty()
    slam_state = StringProperty()

    def __init__(self, **kwargs):
        super(TopicViewPane, self).__init__(**kwargs)
        self.bot_state_sub = rospy.Subscriber(
            "/hullbot/state_machine/current_state", String, callback=self.bot_state_cb
        )
        self.heartbeat_sub = rospy.Subscriber(
            "/Hullbot/bridge/heartbeat/state", State, callback=self.heartbeat_cb
        )
        self.power_board_adc_sub = rospy.Subscriber(
            "/Hullbot/sensors/power_board_adc", PowerBoardADC, callback=self.power_board_adc_cb
        )
        self.rcout_sub = rospy.Subscriber("/Hullbot/bridge/rc/out", RCOut, callback=self.rcout_cb)
        self.cc_feedback_sub = rospy.Subscriber(
            "/hullbot/cleaning_cores/feedback", VescStatusArray, callback=self.cc_feedback_cb
        )
        self.depth_sub = rospy.Subscriber(
            "/Hullbot/bridge/mav_sensor/vfr_hud", VFR_HUD, callback=self.depth_cb
        )
        self.control_mode_sub = rospy.Subscriber(
            "/hullbot/control_mode_handler/control_mode", String, callback=self.control_mode_cb
        )
        self.diagnostics_sub = rospy.Subscriber(
            "/diagnostics", DiagnosticArray, callback=self.diagnostics_cb
        )
        self.rovio_happiness_sub = rospy.Subscriber(
            "/rovio/happiness_accumulated", Float32, callback=self.rovio_happiness_cb
        )
        self.slam_atlas_map_count_sub = rospy.Subscriber(
            "/orbslam/map_count", Int16, callback=self.slam_atlas_map_count_cb
        )
        self.underwater_sub = rospy.Subscriber(
            "/Hullbot/bridge/mav_sensor/underwater", Bool, callback=self.underwater_cb
        )
        self.slam_state_sub = rospy.Subscriber(
            "/orbslam/tracking_state", Int8, callback=self.slam_state_cb
        )

        self.tc_boost_sub = rospy.Subscriber(
            "/Hullbot/control/tc_boost", Bool, callback=self.tc_boost_cb
        )
        self.ping_pub = rospy.Publisher("/hullbot/seawolf/ping", Float32, queue_size=5)

        self.battery_warning_popup = BatteryVoltageWarningPopup()

        # Moving average of the battery voltage
        self.battery_voltage_ave = None
        self.battery_ave_sub = rospy.Subscriber(
            "/Hullbot/sensors/power_board_adc/batt_voltage_avg",
            Float32,
            callback=self.battery_voltage_ave_cb,
        )

        self.warn_popup_shown = False

        # Ping once per second (rospy Timer is threaded so checking ping doesn't block)
        rospy.Timer(rospy.Duration(1), self.ping_cb)

    def slam_state_cb(self, state_int):
        # Slam tracking state
        self.slam_state_text.color = [1, 1, 1, 1]
        if state_int.data == SLAM_SYSTEM_NOT_READY:
            self.slam_state = "SLAM not ready. Check rovio"
            self.slam_state_text.color = [1, 0, 0, 1]
        elif state_int.data == SLAM_NO_IMAGES_YET:
            self.slam_state = "Waiting for images"
            self.slam_state_text.color = [1, 0, 0, 1]
        elif state_int.data == SLAM_NOT_INITIALISED:
            self.slam_state = "Trying to initialise"
            self.slam_state_text.color = [1, 0, 0, 1]
        elif state_int.data == SLAM_OK:
            self.slam_state = "Localised"
            self.slam_state_text.color = [0, 1, 0, 1]
        elif state_int.data == SLAM_RECENTLY_LOST:
            self.slam_state = "Recently lost"
        elif state_int.data == SLAM_LOST:
            self.slam_state = "Lost"
            self.slam_state_text.color = [1, 0, 0, 1]
        else:
            self.slam_state = ""

    def diagnostics_cb(self, diag_arr_msg):
        for status in diag_arr_msg.status:
            if status.name == "jetson_stats load":
                self.cpu_load_str = status.values[0].value
                if float(self.cpu_load_str) > 6:
                    # Red
                    self.cpu_load_num.color = [1, 0, 0, 1]
                elif float(self.cpu_load_str) > 5:
                    # Orange
                    self.cpu_load_num.color = [1, 0.650, 0.141, 1]
                else:
                    # White
                    self.cpu_load_num.color = [1, 1, 1, 1]
            if status.name == "jetson_stats top":
                for pair in status.values:
                    if "rovio_node" in pair.key and "MEM" in pair.value:
                        # Value will be in the form "%MEM: <number>"
                        try:
                            result = float(pair.value.split(" ")[-1])
                            result_str = "%.4f" % result
                            self.rovio_memory_usage = result_str

                            # Colourise label based on desired threshold
                            if result < 30:
                                # White - happy
                                self.rovio_memory_usage_text.color = [1, 1, 1, 1]
                            else:
                                # Red - unhappy
                                self.rovio_memory_usage_text.color = [1, 0, 0, 1]

                        except Exception as e:
                            print("Getting Rovio MEM failed: {}".format(e))
                            rospy.logwarn("Getting Rovio MEM failed: {}".format(e))

                            self.rovio_memory_usage = "-"
                            self.rovio_memory_usage_text.color = [1, 1, 1, 1]

    def rovio_happiness_cb(self, happiness_float):
        # Happiness is 0 - 1
        self.rovio_happiness = str(round(happiness_float.data, 1))
        if happiness_float.data < 0.5:
            # Red - unhappy
            self.rovio_happiness_number_text.color = [1, 0, 0, 1]
        else:
            # Green - happy
            self.rovio_happiness_number_text.color = [0, 1, 0, 1]

    def slam_atlas_map_count_cb(self, map_number):
        self.slam_atlas_map_count = str(map_number.data)

    def underwater_cb(self, is_underwater):
        if is_underwater.data:
            self.underwater_box_col = [0.349, 0.831, 0.745, 0.9]
            self.is_underwater = "Underwater"
        else:
            self.underwater_box_col = [0.858, 0.941, 0.980, 0.9]
            self.is_underwater = "In air"

    def ping_cb(self, timer):
        """
        Check ping and publish
        """
        try:
            output = subprocess.check_output(
                "ping -c 1 192.168.2.2", shell=True, universal_newlines=True
            )
            i1 = output.find("time=") + 5
            i2 = output.find(" ", i1)

            # find returns -1 if the substring is not found
            if i1 == -1 or i2 == -1:
                self.ping = "None"
                self.ping_pub.publish(Float32(float("nan")))
            else:
                self.ping = output[i1:i2]
                if float(self.ping) > 500:
                    # Red
                    self.ping_number_label.color = [1, 0, 0, 1]
                elif float(self.ping) > 100:
                    # Orange
                    self.ping_number_label.color = [1, 0.650, 0.141, 1]
                else:
                    # White
                    self.ping_number_label.color = [1, 1, 1, 1]
                self.ping_pub.publish(Float32(float(self.ping)))

        except subprocess.CalledProcessError:
            self.ping = "None"
            self.ping_pub.publish(Float32(float("nan")))

    def depth_cb(self, vfr_hud):
        self.depth = str(round(vfr_hud.altitude, 3))

    def power_board_adc_cb(self, pwr_adc):
        self.tc_curr_1 = str(round(pwr_adc.tc_current[0], 3))
        self.tc_curr_2 = str(round(pwr_adc.tc_current[1], 3))
        self.tc_curr_3 = str(round(pwr_adc.tc_current[2], 3))
        self.tc_curr_4 = str(round(pwr_adc.tc_current[3], 3))
        self.tc_curr_5 = str(round(pwr_adc.tc_current[4], 3))
        self.tc_curr_6 = str(round(pwr_adc.tc_current[5], 3))
        self.tc_curr_7 = str(round(pwr_adc.tc_current[6], 3))
        self.tc_curr_8 = str(round(pwr_adc.tc_current[7], 3))

        self.cc_curr_1 = str(round(pwr_adc.cc_current[0], 3))
        self.cc_curr_2 = str(round(pwr_adc.cc_current[1], 3))
        self.cc_curr_3 = str(round(pwr_adc.cc_current[2], 3))
        self.cc_curr_4 = str(round(pwr_adc.cc_current[3], 3))

        self.bat_voltage = "{:8.3f}".format(pwr_adc.vbatt_voltage)

        self.bat_current = "{:8.3f}".format(pwr_adc.vbatt_current)

        if pwr_adc.vbatt_current > CURRENT_THRESHOLD:
            self.current_led_label.status = False
        else:
            self.current_led_label.status = True

        if pwr_adc.vbatt_current > 30:
            # Red
            self.current_led_label.color = [1, 0, 0, 1]
        elif pwr_adc.vbatt_current > 20:
            # Orange
            self.current_led_label.color = [1, 0.650, 0.141, 1]
        else:
            # White
            self.current_led_label.color = [1, 1, 1, 1]

    def battery_voltage_ave_cb(self, battery_ave):
        """Get battery average voltage and display a warning if it is low"""

        self.battery_voltage_ave = battery_ave.data
        if (
            self.battery_voltage_ave
            and self.battery_voltage_ave < self.battery_warning_popup.get_warn_voltage()
        ):
            # Display battery voltage warning popup
            if not self.warn_popup_shown:
                self.battery_warning_popup.open()
                # Stop it from showing again
                self.warn_popup_shown = True

            self.voltage_led_label.status = False
            # red
            self.voltage_led_label.color = [1, 0, 0, 1]
        else:
            self.voltage_led_label.status = True
            # white
            self.voltage_led_label.color = [1, 1, 1, 1]

    def bot_state_cb(self, bot_state):
        self.bot_state = json.loads(bot_state.data)["state"]

    def heartbeat_cb(self, state):
        self.arm_status_led_label.status = state.armed
        self.armed = str(self.arm_status_led_label.status)

        if state.armed:
            # Green armed
            self.arm_status_led_label.color = [0, 1, 0, 1]
        else:
            # Red unarmed
            self.arm_status_led_label.color = [1, 0, 0, 1]
        self.mode = state.mode
        self.connected = str(state.connected)
        if state.connected:
            # White: connected, it's the usual state
            self.conn_status_led_label.color = [1, 1, 1, 1]
        else:
            # Red: disconnected
            self.conn_status_led_label.color = [1, 0, 0, 1]

    def rcout_cb(self, rc):
        self.tc_pwm_1 = str(rc.channels[0])
        self.tc_pwm_2 = str(rc.channels[1])
        self.tc_pwm_3 = str(rc.channels[2])
        self.tc_pwm_4 = str(rc.channels[3])
        self.tc_pwm_5 = str(rc.channels[4])
        self.tc_pwm_6 = str(rc.channels[5])
        self.tc_pwm_7 = str(rc.channels[6])
        self.tc_pwm_8 = str(rc.channels[7])

    def cc_feedback_cb(self, msg):
        duty = [0.0] * 4
        rpm = [0] * 4
        for status in msg.status:
            duty[status.vesc_id - 1] = status.duty_cycle
            rpm[status.vesc_id - 1] = status.rpm

        self.cleaning_star_1_rpm = str(rpm[0])
        self.cleaning_port_2_rpm = str(rpm[1])
        self.cleaning_star_3_rpm = str(rpm[2])
        self.cleaning_port_4_rpm = str(rpm[3])

        self.cleaning_star_1_duty = str(round(duty[0], 2))
        self.cleaning_port_2_duty = str(round(duty[1], 2))
        self.cleaning_star_3_duty = str(round(duty[2], 2))
        self.cleaning_port_4_duty = str(round(duty[3], 2))

    def control_mode_cb(self, mode):
        self.control_mode = mode.data

    def tc_boost_cb(self, active):
        if active.data:
            self.tc_boost = "Active"
            self.tc_boost_led_label.status = True
        else:
            self.tc_boost = "Inactive"
            self.tc_boost_led_label.status = False
        self.tc_boost_led_label.set_bool_colour()


if __name__ == "__main__":
    # This won't run just yet
    class WidgetApp(App):
        def build(self):
            return TopicViewPane()

    WidgetApp().run()
