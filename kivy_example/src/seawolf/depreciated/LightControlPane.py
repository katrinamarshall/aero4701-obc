import os

import kivy

kivy.require("1.4.1")
import rospy
from ButtonsControl import LightControl
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("lightControlPane.kv"))


class LightControlPane(BoxLayout):
    def __init__(self, **kwargs):
        super(LightControlPane, self).__init__(**kwargs)

        self.light_control = LightControl.get_instance()

        self.light_buttons = [self.ids.lights_front, self.ids.lights_top]

    def set_all_light_brightnesses(self, val):
        self.light_control.set_all_light_brightnesses(val)

    def toggle_light_group(self, group):
        self.light_control.toggle_light_group(group)


if __name__ == "__main__":

    class WidgetApp(App):
        def __init__(self, **kwargs):
            super(WidgetApp, self).__init__(**kwargs)

        def build(self):
            return LightControlPane()

    WidgetApp().run()
