# Shows a graphical battery display along with percentage and time remaining

from __future__ import division

import os

import kivy
import rospy
from kivy.app import App
from kivy.graphics import Color, RoundedRectangle
from kivy.lang import Builder
from kivy.properties import (
    ListProperty,
    NumericProperty,
    ObjectProperty,
    StringProperty,
)
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.popup import Popup
from kivy.uix.widget import Widget
from std_msgs.msg import Float32

kivy.require("1.4.1")

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("batterydisplay.kv"))


class BatteryDisplay(BoxLayout):
    """
    This class provides a widget which can be added into a UI to display the battery percentage,
    time remaining and a graphical BatteryBar.
    Percentage and time remaining are subscribed to on two different rostopics as seen in the init
    """

    # Threshold for warning popup
    BATT_WARN_PERCENT = 5

    # Let kivy know about the variable so it automatically updates
    batt_percent_str = StringProperty()
    batt_time = StringProperty()

    def __init__(self, **kwargs):
        self.batt_percent_str = "Unknown"
        try:
            super(BatteryDisplay, self).__init__(**kwargs)
        except Exception as e:
            print(e)
        self.batt_percent_sub = rospy.Subscriber(
            "/Hullbot/sensors/power_board_adc/batt_percent", Float32, callback=self.batt_percent_cb
        )
        self.batt_time_mins_pub = rospy.Subscriber(
            "/Hullbot/sensors/power_board_adc/batt_time_mins",
            Float32,
            callback=self.batt_time_mins_cb,
        )

        self.warn_popup_shown = False

    def batt_percent_cb(self, batt_percent):
        """
        Callback for new battery percentage information
        Updates BatteryBar and UI text
        """

        if batt_percent.data >= 0 and batt_percent.data <= 100:
            self.batt_percent = batt_percent.data
            self.batt_percent_str = str(int(self.batt_percent)) + "%"

            try:
                self.battery_bar.update_value(self.batt_percent)
            except Exception as e:
                print("Exception: ", e)
        else:
            self.batt_percent_str = "Unknown"
            rospy.logwarn_throttle(
                30.0, "Error: unexpected battery percentage: {}".format(batt_percent.data)
            )
            self.battery_bar.update_value(0)

    def batt_time_mins_cb(self, batt_time_total_mins):
        """
        Callback for new battery time remaining information
        Updates UI text
        """

        if batt_time_total_mins.data == -1.0:
            self.batt_time = "-1h -1m"
            return

        # Find battery time left as hours and minutes
        batt_time_mins = int(batt_time_total_mins.data) % 60
        batt_time_hrs = int(batt_time_total_mins.data / 60)
        self.batt_time = "{}h {}m".format(batt_time_hrs, batt_time_mins)


class BatteryBar(Widget):
    """
    This class implements a graphical display to show the battery percentage.
    The bar goes from full to empty and changes colour from green to yellow to orange
    to red as appropriate.
    """

    # Scale factor of rectangle height for battery bar
    scaled_value = NumericProperty(0.0)
    value = NumericProperty(0.0)
    # Colour of rectangle for battery bar
    batt_color = ListProperty([0, 1, 0.5, 1])
    batt_bat_round_rect = ObjectProperty()

    def __init__(self, **kwargs):
        try:
            super(BatteryBar, self).__init__(**kwargs)
        except Exception as e:
            print(e)
        # Asset file names
        battery_file = "../assets/battery_icon.png"
        self.battery_path = os.path.join(os.path.dirname(__file__), battery_file)

        # Initialise coloured battery bar on canvas
        with self.canvas:
            self.batt_bar_col = Color(0, 0, 0, 0)
            self.batt_bar_rect = RoundedRectangle()

        # Make it update on resize
        self.bind(pos=self.update_display, size=self.update_display)

    def update_display(self, *args):  # *args is needed for kivy bind
        """
        Update the graphical display with the current battery percentage
        Adjusts length and colour of the bar
        """
        self.scaled_value = self.value / 100

        # Determine battery colour
        if self.value <= 20 and self.value >= 0:
            # red
            batt_color = [1, 0, 0, 1]
        elif self.value > 20 and self.value <= 50:
            # orange
            batt_color = [1, 0.5, 0, 1]
        elif self.value > 50 and self.value <= 80:
            # yellow
            batt_color = [1, 1, 0, 1]
        elif self.value > 80 and self.value <= 100:
            # green
            batt_color = [0, 1, 0, 1]
        else:
            # Unknown - out of bounds
            batt_color = [0, 0, 0, 1]

        # Update the final values
        self.batt_bar_col.rgba = batt_color
        self.batt_bar_rect.size = ((0.9 * (self.width) * (self.scaled_value)), self.height)
        self.batt_bar_rect.pos = self.pos

    def update_value(self, value):
        """
        Update the display with a new battery percentage
        """
        self.value = value
        self.update_display()


class BatteryVoltageWarningPopup(Popup):
    """
    A popup to warn the user of low battery
    """

    # The value to display in "Battery below {} volts"
    warn_voltage = NumericProperty()

    def __init__(self, warn_voltage=21.2, **kwargs):
        # Kivy
        super(BatteryVoltageWarningPopup, self).__init__(**kwargs)
        # Update the text for the popup
        self.warn_voltage = warn_voltage

    def get_warn_voltage(self):
        return self.warn_voltage


if __name__ == "__main__":

    class WidgetApp(App):
        def __init__(self, **kwargs):
            super(WidgetApp, self).__init__(**kwargs)

        def build(self):
            self.my_batt = BatteryDisplay()
            return BatteryDisplay()

    rospy.init_node("batt_test")
    WidgetApp().run()
