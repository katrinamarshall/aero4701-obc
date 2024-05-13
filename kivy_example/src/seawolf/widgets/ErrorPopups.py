import subprocess

import kivy

kivy.require("1.2.0")
import os

import rospy
from ButtonsControl import RovioControl
from diagnostic_msgs.msg import DiagnosticArray
from dismissable_popup import show_dismissable_popup
from kivy.app import App
from kivy.lang import Builder
from std_msgs.msg import Float32

kivy.require("1.2.0")


class ErrorPopups:
    def __init__(self, **kwargs):
        self.diagnostics_sub = rospy.Subscriber(
            "/diagnostics", DiagnosticArray, callback=self.diagnostics_cb
        )
        rospy.Subscriber("/hullbot/seawolf/ping", Float32, callback=self.ping_cb)
        self.rovio_control = RovioControl.get_instance()

        self.ping_counter = 0
        self.memory_popup_open = False

    def diagnostics_cb(self, diag_arr_msg):
        for status in diag_arr_msg.status:
            if status.name == "jetson_stats top":
                for pair in status.values:
                    if "rovio_node" in pair.key and "MEM" in pair.value:
                        # Value will be in the form "%MEM: <number>"
                        try:
                            result = float(pair.value.split(" ")[-1])
                            result_str = "%.4f" % result
                            self.rovio_memory_usage = result_str
                            if not self.memory_popup_open and result >= 30:
                                self.Rovio_mem_leak_popup(result)
                            elif self.memory_popup_open and result < 10:
                                self.memory_popup_open = False

                        except Exception as e:
                            print("Getting Rovio MEM failed: {}".format(e))
                            rospy.logwarn("Getting Rovio MEM failed: {}".format(e))

    def Rovio_mem_leak_popup(self, percentage):
        self.memory_popup_open = True
        popup_title = "Warning: High Rovio Memory Use"
        popup_text = "Rovio is using " + str(percentage) + "% of avaliable memory"
        popup_ok_button_text = "Restart"

        self.memory_popup = show_dismissable_popup(
            popup_title,
            popup_text,
            ok_button_fn=self.Restart_rovio,
            ok_button_text=popup_ok_button_text,
        )

    def Restart_rovio(self, *args):
        """
        Start rovio using remote supervisor
        """
        if self.memory_popup:
            self.memory_popup.dismiss()
        self.rovio_control.start_rovio()

    def ping_cb(self, ping):
        if ping.data >= 250:
            # If the ping goes over 250, iterate a counter
            self.ping_counter += 1
            # When that counter hits 5 consecutive readings of over 250, open the popup
            if self.ping_counter >= 5:
                self.high_ping_popup()
                self.ping_counter = 0
        else:
            # If the ping goes below 250, reset the counter
            self.ping_counter = 0

    def high_ping_popup(self):
        popup_title = "Warning: High Ping"
        popup_text = "Ping is consistantly above 250 ms."
        popup_text += "\nCheck the LaptopSide and restart."
        popup_text += "\nIf the issue persists run further diagnostics"

        show_dismissable_popup(title=popup_title, text=popup_text)


if __name__ == "__main__":

    class WidgetApp(App):
        def build(self):
            return ErrorPopups()

    WidgetApp().run()
