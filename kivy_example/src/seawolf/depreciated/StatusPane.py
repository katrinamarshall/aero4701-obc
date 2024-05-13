import json
import os
import sys

import kivy
import rospy

kivy.require("1.2.0")
from BatteryDisplay import BatteryVoltageWarningPopup
from hullbot_msgs.msg import PowerBoardADC
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import StringProperty
from kivy.uix.boxlayout import BoxLayout
from mavros_msgs.msg import VFR_HUD, State
from NodeIndicatorsPilotView import NodeIndicatorsPilotView
from std_msgs.msg import Bool, Float32, String

CURRENT_THRESHOLD = 50.0

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("statusPane.kv"))


class InfoPanePilotView(BoxLayout):
    system_armed = StringProperty("DISARMED")

    bot_state = StringProperty("-")
    control_mode = StringProperty("")
    tc_boost = StringProperty("TC BOOST INACTIVE")
    depth = StringProperty("Unknown")
    underwater_state = None
    rovio_happiness = StringProperty("-")
    bat_voltage = StringProperty()
    bat_current = StringProperty()

    def __init__(self, **kwargs):
        super(InfoPanePilotView, self).__init__(**kwargs)

        # Subscribers
        self.heartbeat_sub = rospy.Subscriber(
            "/Hullbot/bridge/heartbeat/state", State, callback=self.armed_cb
        )
        self.bot_state_sub = rospy.Subscriber(
            "/hullbot/state_machine/current_state", String, callback=self.bot_state_cb
        )
        self.control_mode_sub = rospy.Subscriber(
            "/hullbot/control_mode_handler/control_mode", String, callback=self.control_mode_cb
        )
        self.tc_boost_sub = rospy.Subscriber(
            "/Hullbot/control/tc_boost", Bool, callback=self.tc_boost_cb
        )
        self.depth_sub = rospy.Subscriber(
            "/Hullbot/bridge/mav_sensor/vfr_hud", VFR_HUD, callback=self.depth_cb
        )
        self.underwater_sub = rospy.Subscriber(
            "/Hullbot/bridge/mav_sensor/underwater", Bool, callback=self.underwater_cb
        )
        self.power_board_adc_sub = rospy.Subscriber(
            "/Hullbot/sensors/power_board_adc", PowerBoardADC, callback=self.power_board_adc_cb
        )

        # Add widgets
        self.ids.node_indicator_container.add_widget(NodeIndicatorsPilotView())
        self.battery_warning_popup = BatteryVoltageWarningPopup()
        self.warn_popup_shown = False

        # Moving average of the battery voltage
        self.battery_voltage_ave = None
        self.battery_ave_sub = rospy.Subscriber(
            "/Hullbot/sensors/power_board_adc/batt_voltage_avg",
            Float32,
            callback=self.battery_voltage_ave_cb,
        )

        # Set initial states of indicators
        self.ids.bot_armed_indicator.set_bad()

    def armed_cb(self, state):
        if state.armed:
            self.system_armed = "ARMED"
            self.ids.bot_armed_indicator.set_good()
        else:
            self.system_armed = "DISARMED"
            self.ids.bot_armed_indicator.set_bad()

    def bot_state_cb(self, bot_state):
        self.bot_state = json.loads(bot_state.data)["state"]

    def control_mode_cb(self, mode):
        self.control_mode = mode.data

    def tc_boost_cb(self, active):
        if active.data:
            self.tc_boost = "BOOST ACTIVE"
            self.ids.tc_boost_indicator.set_on()
        else:
            self.tc_boost = "BOOST INACTIVE"
            self.ids.tc_boost_indicator.set_neutral()

    def depth_cb(self, vfr_hud):
        if not self.underwater_state and float(vfr_hud.altitude) > -0.1:
            self.depth = "NOT UNDERWATER"
        else:
            self.depth = str(round(vfr_hud.altitude, 3)) + "M"

    def underwater_cb(self, is_underwater):
        self.underwater_state = is_underwater.data

    def rovio_happiness_cb(self, happiness_float):
        # Happiness is 0 - 1
        self.rovio_happiness = str(round(happiness_float.data, 1))
        if happiness_float.data < 0.5:
            self.ids.rovio_happiness_indicator.set_bad()
        else:
            self.ids.rovio_happiness_indicator.set_good()

    def power_board_adc_cb(self, pwr_adc):
        self.bat_voltage = "{:8.3f}".format(pwr_adc.vbatt_voltage)

        self.bat_current = "{:8.3f}".format(pwr_adc.vbatt_current)

        if pwr_adc.vbatt_current > CURRENT_THRESHOLD:
            self.ids.current_indicator.set_bad()

        if pwr_adc.vbatt_current > 30:
            self.ids.current_indicator.set_bad()
        elif pwr_adc.vbatt_current > 20:
            self.ids.current_indicator.set_warn()
        else:
            self.ids.current_indicator.set_neutral()

    def battery_voltage_ave_cb(self, ave_voltage):
        self.battery_voltage_ave = ave_voltage.data
        if (
            self.battery_voltage_ave
            and self.battery_voltage_ave < self.battery_warning_popup.get_warn_voltage()
        ):
            # Display battery voltage warning popup
            if not self.warn_popup_shown:
                self.battery_warning_popup.open()
                # Stop it from showing again
                self.warn_popup_shown = True

                self.ids.voltage_indicator.set_bad()
            else:
                self.ids.voltage_indicator.set_neutral()
