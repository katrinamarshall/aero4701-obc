import os
import sys

import kivy

kivy.require("1.4.1")
from kivy.app import App
from kivy.clock import Clock
from kivy.graphics import Color, Rectangle
from kivy.lang import Builder
from kivy.properties import BoundedNumericProperty, NumericProperty
from kivy.uix.behaviors import ButtonBehavior
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.popup import Popup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("tofDisplay.kv"))

import dynamic_reconfigure.client
import rospy
import subs
from hullbot_msgs.msg import TOFChain
from std_msgs.msg import Int16MultiArray

TOF_MODE_VALS = {
    0: {"ticks": [0, 20, 40, 60, 80, 100, 120, 140], "colour_bounds": [0, 90, 110, 130]},
    1: {"ticks": [0, 50, 100, 150, 200, 250, 300, 350], "colour_bounds": [0, 120, 240, 340]},
}
TOF_MODE_INIT = 1


class TofDisplay(BoxLayout, ButtonBehavior):
    front_star = NumericProperty(160)
    top_front_star = NumericProperty(160)
    top_back_star = NumericProperty(160)
    front_port = NumericProperty(160)
    top_front_port = NumericProperty(160)
    top_back_port = NumericProperty(160)

    # Intialise TOF mode in the wrong mode so that we can change it back in __init__ and initialise the display
    # XOR with 1 toggles a bit (0 -> 1, 1 -> 0)
    tof_mode = NumericProperty(TOF_MODE_INIT ^ 1)

    def __init__(self, **kwargs):
        super(TofDisplay, self).__init__(**kwargs)
        self.tof_sub = rospy.Subscriber(
            "/hullbot/tof_node/all_tofs_filtered", TOFChain, self.tof_cb
        )

        # We will use ROS dynamic_reconfigure to change the TOF mode in tof_node directly
        self.tofs_node_client = None
        self.first_tof_mode_try = False

        # Run the setup first time to display ticks
        # This triggers all children to have their TOF mode changed (this is the way we have set it up)
        # Thus causing the ticks to be drawn on the TOF scale
        self.tof_mode = TOF_MODE_INIT

        # Initialise the error popup to be shown when errors occur
        self.tof_mode_popup = TOFErrorPopup()

    def tof_cb(self, tof_data):
        """
        Callback for new TOF data from ROS
        """

        # Initialise the dynamic_reconfigure client now that we are getting data
        if self.tofs_node_client is None and not self.first_tof_mode_try:
            self.get_tof_node_client()
            self.first_tof_mode_try = True

        # Get the new data and convert from m to mm
        self.front_star = tof_data.ranges[0] * 1000
        self.top_front_star = tof_data.ranges[1] * 1000
        self.top_back_star = tof_data.ranges[2] * 1000
        self.front_port = tof_data.ranges[3] * 1000
        self.top_front_port = tof_data.ranges[4] * 1000
        self.top_back_port = tof_data.ranges[5] * 1000

    def tofs_config_cb(self, config):
        """
        Callback for a change in TOF mode from dynamic_reconfigure
        """

        if config["tof_mode"] != self.tof_mode:
            self.tof_mode = config["tof_mode"]

    def tof_mode_button_release(self):
        """
        Callback for the TOF mode button
        Use dynamic_reconfigure to change the TOF mode
        """
        # Simply change the TOF mode if we've already set up the client
        if self.tofs_node_client is not None:
            try:
                self.tofs_node_client.update_configuration({"tof_mode": self.tof_mode ^ 1})
            except dynamic_reconfigure.DynamicReconfigureCallbackException as e:
                self.tof_mode_popup.label_text.text = "Dynamic_reconfigure service not found"
                self.tof_mode_popup.error_text.text = repr(e)
                self.tof_mode_popup.open()
                self.tofs_node_client = None
        else:
            # If the client is not yet configured, try to initialise it
            if self.tofs_node_client is None:
                self.get_tof_node_client()

            if self.tofs_node_client is not None:
                # If initialisation successful, change the mode
                self.tofs_node_client.update_configuration({"tof_mode": self.tof_mode ^ 1})

    def get_tof_node_client(self):
        """
        Initialise the dynamic_reconfigure client to get and set the TOF mode
        TOF mode --> 0 = normal, 1 = extended
        """
        # If tof_node cannot be found, we get the following excpetion:
        # ROSException: timeout exceeded while waiting for service /tofs_node/set_parameters
        try:
            self.tofs_node_client = dynamic_reconfigure.client.Client(
                "tof_node", timeout=5, config_callback=self.tofs_config_cb
            )
            print("'tof_node' client found")
        except rospy.ROSException as e:
            print("Could not find 'tof_node' client, assuming extended range\n{}".format(e))
            self.tof_mode_popup.label_text.text = "tof_node client not running"
            self.tof_mode_popup.error_text.text = repr(e)
            self.tof_mode_popup.open()
            # Automatically choose extended range as it can take into account the normal range too
            self.tof_mode = 1
            self.tofs_node_client = None


class TofsBar(BoxLayout):
    """
        When the value gets updated, we would like to make the appropriate
    changes to the bar height and color. The maximum distance measured by the sensor is ~190mm.
    Underwater this equates to about 140mm actual distance.
    As we set the desired TofHold target in 'TOF measurement scale' as opposed to absolute distance,
    we classify the the measurements displayed in the following way:
            d <=  90   - green (optimal range for most hulls)
       90 < d <= 120   - yellow
      120 < d <= 150   - orange
            d >  150   - red

    The rectangle is created and updated in Python. Doing
    it in the kv file instead seems to be buggy and laggy.
    """

    # Keep the value between min and max and bind to a callback
    # on change with kivy
    tof_mode = NumericProperty(TOF_MODE_INIT)
    value = NumericProperty(TOF_MODE_VALS[1]["ticks"][0])
    scaled_value = BoundedNumericProperty(1, min=0, max=1)

    def __init__(self, **kwargs):
        super(TofsBar, self).__init__(**kwargs)

        # Initialise rectangle bar on canvas
        with self.canvas.after:
            self.col = Color(0, 0, 0, 0)
            self.bar_rect = Rectangle()
            self.bar_rect.pos = (self.pos[0], self.pos[1] + self.height * (1 - self.scaled_value))
            self.bar_rect.size = (self.width, self.height * self.scaled_value)

        # Make it update on resize
        self.bind(pos=self.update, size=self.update)

        # Set min and max initially
        self.on_tof_mode()

    def on_value(self, instance, value):
        """
        Callback for value changes (automatically bound by kivy)
        """
        self.update()

    def update(self, *args):
        """
        Draw the rectangle using the current (updated) value
        """

        is_green = (
            self.value <= TOF_MODE_VALS[self.tof_mode]["colour_bounds"][1]
            and self.value >= TOF_MODE_VALS[self.tof_mode]["colour_bounds"][0]
        )
        is_yellow = (
            self.value > TOF_MODE_VALS[self.tof_mode]["colour_bounds"][1]
            and self.value <= TOF_MODE_VALS[self.tof_mode]["colour_bounds"][2]
        )
        is_orange = (
            self.value > TOF_MODE_VALS[self.tof_mode]["colour_bounds"][2]
            and self.value <= TOF_MODE_VALS[self.tof_mode]["colour_bounds"][3]
        )
        is_red = self.value > TOF_MODE_VALS[self.tof_mode]["colour_bounds"][
            3
        ] and self.value < float("inf")
        if is_red:
            color = [1, 0, 0, 1]
        elif is_orange:
            color = [1, 0.5, 0, 1]
        elif is_yellow:
            color = [1, 1, 0, 1]
        elif is_green:
            color = [0, 1, 0, 1]
        else:
            # Set to grey for infinity
            color = [0.5, 0.5, 0.5, 1]

        # Check for infinity
        # Arithmetic on infinity causes a crash
        if self.value > TOF_MODE_VALS[self.tof_mode]["ticks"][-1]:
            self.scaled_value = 1
        elif self.value < TOF_MODE_VALS[self.tof_mode]["ticks"][0]:
            # In case of -inf
            self.scaled_value = 0
        else:
            # Map the value from 0-1 of the maximum
            self.scaled_value = (self.value - self.min) / (self.max - self.min)

        # Update all the values
        self.col.rgba = color
        self.bar_rect.size = (self.width, self.height * self.scaled_value)
        self.bar_rect.pos = (self.pos[0], self.pos[1] + self.height * (1 - self.scaled_value))

    def on_tof_mode(self, *args):
        """
        Callback for tof_mode change, automatically bound by Kivy
        Adjusts the min and max values to scale between
        """

        # Min and max distances of the TOF to display
        self.max = TOF_MODE_VALS[self.tof_mode]["ticks"][-1]
        self.min = TOF_MODE_VALS[self.tof_mode]["ticks"][0]

        # Rescale if the value is now out of range
        if self.value > self.max:
            self.value = float("inf")
        elif self.value < self.min:
            self.value = self.min
        else:
            # This will happen automatically happen for the above two cases due to on_value being run when self.value is changed. If self.value is not changed, we need to run the update function manually to refresh the display in the new TOF mode
            self.update()


class LabelTicks(GridLayout):
    tof_mode = NumericProperty(TOF_MODE_INIT)

    def on_tof_mode(self, *args):
        """
        Update the tick values on tof mode change
        """
        # The ticks happen to go in reverse order of low to high
        i = len(TOF_MODE_VALS[self.tof_mode]["ticks"]) - 1
        for child in self.children:
            # Look for every Label object in the LabelTicks and change its value
            if type(child).__name__ == "Label":
                child.text = str(TOF_MODE_VALS[self.tof_mode]["ticks"][i])
                i -= 1


class TofsScale(BoxLayout):
    tof_mode = NumericProperty(TOF_MODE_INIT)


class Tick(BoxLayout):
    pass


class TickTop(BoxLayout):
    pass


class TOFErrorPopup(Popup):
    pass


if __name__ == "__main__":

    class WidgetApp(App):
        def __init__(self, **kwargs):
            super(WidgetApp, self).__init__(**kwargs)
            self.my_vals = [60, 90, 120, 150]
            rospy.init_node("tofs_kivy_node")

        def build(self):
            self.my_tofs = TofDisplay(values=self.my_vals)
            Clock.schedule_interval(self.update_values, 0.1)
            self.my_tofs = TofDisplay()

            return self.my_tofs

        # spoofed data
        def update_values(self, dt):
            for i in range(len(self.my_vals)):
                v = self.my_vals[i]
                v += 1.0
                if v >= 160:
                    v = 0.0
                self.my_vals[i] = v
            self.my_tofs.values = self.my_vals

    WidgetApp().run()
