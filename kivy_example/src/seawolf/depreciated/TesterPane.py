import os

import kivy

kivy.require("1.2.0")

from kivy.lang import Builder
from kivy.uix.tabbedpanel import TabbedPanel

import seawolf
from seawolf.widgets.BotConfigurationPane import BotConfigurationPane
from seawolf.widgets.CalibrationPane import CalibrationPane
from seawolf.widgets.MotorTesterPane import MotorTesterPane
from seawolf.widgets.PodControlPane import PodControlPane
from seawolf.widgets.PodDisplayPane import PodDisplayPane

Builder.load_file(seawolf.form_kivy_config_path("testerPane.kv"))


class TesterPane(TabbedPanel):
    def __init__(self):
        super(TesterPane, self).__init__()
        self.ids.bot_configuration_container.add_widget(BotConfigurationPane())
        self.ids.calibration_container.add_widget(CalibrationPane())
        self.ids.motor_tester_container.add_widget(MotorTesterPane())
        self.ids.pod_control_container.add_widget(PodControlPane())
        self.ids.pod_display_container.add_widget(PodDisplayPane())
