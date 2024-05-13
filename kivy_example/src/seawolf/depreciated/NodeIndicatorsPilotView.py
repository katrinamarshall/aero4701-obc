import os

import kivy
import rospy

kivy.require("1.2.0")
from kivy.app import App
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.properties import StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.popup import Popup
from NodeStatusPane import NodeIndicatorGroup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("nodeIndicatorsPilotView.kv"))


class NodeIndicatorsPilotView(BoxLayout):
    """
    Indicator container for all node status' including the ability to expand through a popup to show individual nodes
    """

    system_status = StringProperty("-")

    def __init__(self, **kwargs):
        super(NodeIndicatorsPilotView, self).__init__(**kwargs)

        self.node_popup = NodeIndicatorsPopup()
        self.node_status = self.node_popup.get_node_group()

        # Clock for continual update
        Clock.schedule_interval(self.update_overall_status, 0.25)

    def update_overall_status(self, dt):
        status = self.node_status.get_status()
        if status:
            self.ids.node_indicator_button.set_true()
            self.system_status = "System running"
        else:
            self.ids.node_indicator_button.set_neutral()
            self.system_status = "Node down"

    def on_indicator(self):
        self.node_popup.open()


class NodeIndicatorsPopup(Popup):
    """
    Popup revealing individual node status'
    """

    def __init__(self, **kwargs):
        super(NodeIndicatorsPopup, self).__init__(**kwargs)

        self.node_group = NodeIndicatorGroup.get_instance(self.ids.indicator_container)

    def get_node_group(self):
        return self.node_group
