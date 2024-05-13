from kivy.uix.boxlayout import BoxLayout

from seawolf.widgets.ArmedIndicator import ArmedIndicator
from seawolf.widgets.BatteryIndicator import BatteryIndicator
from seawolf.widgets.CleaningIndicator import CleaningIndicator
from seawolf.widgets.ControlPane import ControlPane
from seawolf.widgets.DepthGraph import DepthGraph
from seawolf.widgets.ErrorPopups import ErrorPopups
from seawolf.widgets.RollIndicator import RollIndicator
from seawolf.widgets.SpeedIndicator import SpeedIndicator
from seawolf.widgets.StateIndicator import StateIndicator
from seawolf.widgets.TofIndicator import TofIndicator


class FullScreen(BoxLayout):
    def __init__(self, **kwargs):
        super(FullScreen, self).__init__(**kwargs)

        # Popup Widgets
        self.errors = ErrorPopups()

        # Overlay Widgets
        self.ids.tof_container.add_widget(TofIndicator())
        self.ids.speed_container.add_widget(SpeedIndicator())
        self.ids.cleaning_container.add_widget(CleaningIndicator())
        self.ids.depth_container.add_widget(DepthGraph())

        self.ids.power_container.add_widget(BatteryIndicator())
        self.ids.roll_container.add_widget(RollIndicator())

        # Text Widgets
        self.ids.armed_container.add_widget(ArmedIndicator())
        self.ids.mode_container.add_widget(StateIndicator())

        # Right pullout panel
        self._right = self.ids.right_container
        self._right.bind(minimum_height=self._right.setter("height"))
        self._right.add_widget(ControlPane())

    def set_view_type(self, view_type):
        self.ids.view_type.clear_widgets()
        self.ids.view_type.add_widget(view_type)
