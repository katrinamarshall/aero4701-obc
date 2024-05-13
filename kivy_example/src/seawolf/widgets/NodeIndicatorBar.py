import kivy
import rospy
from kivy.clock import Clock
from std_msgs.msg import Bool

kivy.require("1.2.0")
import rosnode
from dismissable_popup import show_dismissable_popup
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ColorProperty, ListProperty
from kivy.uix.boxlayout import BoxLayout

import seawolf
from seawolf.utils.customWidgets import CustomTextIndicator

Builder.load_file(seawolf.form_kivy_config_path("nodeIndicatorBar.kv"))


class NodeIndicatorBar(BoxLayout):
    """Simple node indicator by polling active nodes every second.

    TO-DO: Use diagnostics or complex logic to figure out more specific node states
    """

    def __init__(self, **kwargs):
        super(NodeIndicatorBar, self).__init__(**kwargs)
        self.node_indicators = []
        self._add_node_indicator(
            NodeIndicator(name="Cameras", node_name="/perception/camera_manager")
        )
        self._add_node_indicator(
            NodeIndicator(name="Motion Controller", node_name="/motion/controller_manager")
        )
        self._add_node_indicator(
            NodeIndicator(name="Localisation", node_name="/localisation/fg_odom")
        )
        self._add_node_indicator(
            NodeIndicator(name="Mission Manager", node_name="/mission/mission_manager")
        )
        self._add_node_indicator(
            NodeIndicator(name="Bag Cont.", node_name="/rosbag/continuous/recorder")
        )
        self._add_node_indicator(
            NodeIndicator(name="Bag Mission", node_name="/rosbag/mission/recorder")
        )

        update_rate = 1  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))

    def _add_node_indicator(self, node_indicator):
        """
        Add node indicator to widget.
        """
        self.node_indicators.append(node_indicator)
        self.ids.node_indicators.add_widget(node_indicator)

    def _update_graphics_cb(self, *args):
        """
        Update the node indicator to active or dead based on avaliable nodes.
        """
        active_nodes = rosnode.get_node_names()
        for node in self.node_indicators:
            node_name = node.get_node_name()
            if node_name in active_nodes:
                node.set_active()
            elif node.check_once_active():
                node.set_dead()


class NodeIndicator(CustomTextIndicator):
    def __init__(self, name, node_name, warn=False):
        super(NodeIndicator, self).__init__()
        self._name = name
        self._node_name = node_name
        self._once_active = False
        self.set_text(self._name)
        self._warn = warn
        self._alive = False

    def get_node_name(self):
        return self._node_name

    def check_once_active(self):
        return self._once_active

    def set_active(self):
        self.set_success()
        self._alive = True
        if not self._once_active:
            self._once_active = True

    def set_dead(self):
        self.set_error()
        if self._alive and self._warn:
            show_dismissable_popup(title="Error", text="{} has died!".format(self._name))
        self._alive = False
