import copy
from collections import deque

import kivy
import rospy
import math

from kivy.clock import Clock
from sensor_msgs.msg import BatteryState


kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ColorProperty, NumericProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from WarningPopups import BatteryWarningPopup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("batteryIndicator.kv"))

WARNING_THRESHOLD = 37


class BatteryIndicator(BoxLayout):
    """A widget to display battery voltage."""

    display_voltage = NumericProperty(0)
    display_percentage = NumericProperty(0)
    display_voltage_text = StringProperty("0 V")

    def __init__(self, **kwargs):
        super(BatteryIndicator, self).__init__(**kwargs)

        self._timeout = 5
        self._last_cb_time = 0
        self._low_batt_percentage = 0.1

        self.batt_voltage = float("nan")
        self.batt_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self.batt_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        self.batt_percentage = float("nan")
        self.batt_text = self._convert_batt_status_to_text()

        self._battery_sub = rospy.Subscriber(
            "~battery_monitor", BatteryState, self._battery_status_cb
        )

        self._battery_warning_popup = BatteryWarningPopup(WARNING_THRESHOLD)

        update_rate = 1  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))

    def _battery_status_cb(self, msg):
        """Callback for battery status.

        Args:
            msg (BatteryState): BatteryState message
        """
        self._last_cb_time = rospy.get_time()
        self.batt_voltage = msg.voltage
        self.batt_status = msg.power_supply_status
        self.batt_health = msg.power_supply_health
        self.batt_percentage = msg.percentage

    def _update_graphics_cb(self, *args):
        """Update text, text colour and slider value based on average voltage."""
        if self._battery_topic_timeout():
            self._set_display_to_timeout()
        elif self.batt_health == BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN:
            self._set_display_to_timeout()
        elif self.batt_health == BatteryState.POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE:
            self._set_display_to_timeout()
        elif self.batt_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD:
            self._set_display_to_battery_dead()
            self._battery_warning_popup.show()
        elif self.batt_health == BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE:
            self._set_display_to_battery_overvoltage()
        elif (
            self.batt_health == BatteryState.POWER_SUPPLY_HEALTH_GOOD
            and self.batt_percentage < self._low_batt_percentage
        ):
            self._set_display_to_battery_low()
        elif self.batt_health == BatteryState.POWER_SUPPLY_HEALTH_GOOD:
            self._set_display_to_battery_ok()
        else:
            self._set_display_to_battery_ok()

    def _set_display_to_timeout(self):
        # Make the slider be at minimum but set text color to white.
        self.batt_voltage = float("nan")
        self.batt_percentage = float("nan")
        self.batt_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self.batt_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        self.batt_text = self._convert_batt_status_to_text()
        self.ids.voltage_text.set_unresponsive()
        self.display_percentage = 0.0
        self.display_voltage = 0.0
        self.display_voltage_text = self.batt_text

    def _set_display_to_battery_dead(self):
        self.batt_text = self._convert_batt_status_to_text()
        self.display_percentage = self.batt_percentage
        self.display_voltage = self.batt_voltage
        self.display_voltage_text = self.batt_text
        self.ids.voltage_text.set_error()

    def _set_display_to_battery_ok(self):
        self.batt_text = self._convert_batt_status_to_text()
        self.display_percentage = self.batt_percentage
        self.display_voltage = self.batt_voltage
        self.display_voltage_text = self.batt_text
        self.ids.voltage_text.set_neutral()

    def _set_display_to_battery_overvoltage(self):
        self.batt_text = self._convert_batt_status_to_text()
        self.display_percentage = self.batt_percentage
        self.display_voltage = self.batt_voltage
        self.display_voltage_text = self.batt_text
        self.ids.voltage_text.set_error()

    def _set_display_to_battery_low(self):
        self.batt_text = self._convert_batt_status_to_text()
        self.display_percentage = self.batt_percentage
        self.display_voltage = self.batt_voltage
        self.display_voltage_text = self.batt_text
        self.ids.voltage_text.set_warn()

    def _convert_batt_status_to_text(self):
        """Convert voltage to text.

        Args:
            voltage (float): Voltage

        Returns:
            string: Voltage text
        """
        if math.isnan(self.batt_voltage):
            return "0 V - No Data"
        elif self.batt_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD:
            return str(round(self.batt_voltage, 2)) + " V - Battery Dead!"
        elif self.batt_health == BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE:
            return str(round(self.batt_voltage, 2)) + " V - Battery Overvoltage!"
        elif (
            self.batt_health == BatteryState.POWER_SUPPLY_HEALTH_GOOD
            and self.batt_percentage < self._low_batt_percentage
        ):
            return str(round(self.batt_voltage, 2)) + " V - Battery Low"
        else:
            return str(round(self.batt_voltage, 2)) + " V"

    def _battery_topic_timeout(self):
        """Check timeout between current time and msg recieved time.

        Returns:
            Bool: timeout flag
        """
        now = rospy.get_time()
        return (now - self._last_cb_time) > self._timeout
