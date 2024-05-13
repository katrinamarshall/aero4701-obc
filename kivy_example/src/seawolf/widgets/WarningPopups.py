import kivy

kivy.require("1.2.0")
from kivy.lang import Builder
from kivy.uix.label import Label
from kivy.uix.popup import Popup

import seawolf
from seawolf.utils.customWidgets import CustomTextIndicator

Builder.load_file(seawolf.form_kivy_config_path("warningPopups.kv"))


class WarningPopup(Popup):
    """A popup to warn the user of suspicious depth readings."""

    def __init__(self, text, max_shown):
        super(WarningPopup, self).__init__()
        self._text = text
        self._num_shown = 0
        self._max_shown = max_shown
        self.add_label()

    def add_label(self):
        # Cannot resize CustomTextIndicator inside Popup so using default Label
        self.content = Label(
            text=self._text, color=[1, 0, 0, 1], font_name="formular", font_size="25sp"
        )

    def show(self):
        if self._num_shown < self._max_shown:
            self.open()
            self._num_shown += 1


class BatteryWarningPopup(WarningPopup):
    """A popup to warn the user of low battery."""

    def __init__(self, warn_voltage):
        text = "Low battery under {} V!\nShut down bot immediately!".format(warn_voltage)
        super(BatteryWarningPopup, self).__init__(text=text, max_shown=3)


class DepthWarningPopup(WarningPopup):
    """A popup to warn the user of suspicious depth readings."""

    def __init__(self, warn_depth):
        text = (
            "Suspicious depth reading above {} m!\nCheck barometer and odometry solution!".format(
                warn_depth
            )
        )
        super(DepthWarningPopup, self).__init__(text=text, max_shown=3)
