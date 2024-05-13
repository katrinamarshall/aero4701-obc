import kivy

kivy.require("1.2.0")
from kivy.lang import Builder
from kivy.properties import BooleanProperty, ListProperty
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.widget import Widget

Builder.load_string(
    """
<Led>:
    canvas:
        Color:
            rgba: root.color
        Rectangle:
            size: self.size
            pos: self.pos
"""
)


class Led(Widget):
    # Although it goes against every fibre of my being, kivy uses the American
    # spelling, so to be consistent we do too
    color = ListProperty([1, 1, 1, 0.4])  # set grey as default
    status = BooleanProperty(False)

    def set_grey(self):
        self.color = [1, 1, 1, 0.4]  # set grey

    def set_blue(self):
        self.color = [0.17, 0.3, 1, 0.5]  # set blue

    def set_color(self, color):
        self.color = color

    def set_bool_colour(self):
        if self.status:
            return self.set_blue()
        else:
            return self.set_grey()


class RedGreenLedLabel(Label):
    color = ListProperty([1, 1, 1, 0.4])  # set grey as default
    status = BooleanProperty(False)

    def set_red(self):
        self.color = [1, 0.1, 0.2, 0.5]  # set red

    def set_green(self):
        self.color = [0, 1, 0.1, 0.5]  # set green

    def set_color(self, color):
        self.color = color

    def set_bool_colour(self):
        if self.status:
            return self.set_green()
        else:
            return self.set_red()


class RedWarnLedLabel(Label):
    color = ListProperty([1, 1, 1, 0.4])  # set grey as default
    status = BooleanProperty(False)

    def set_red(self):
        self.color = [1, 0.1, 0.2, 0.5]  # set red

    def set_white(self):
        self.color = [1, 1, 1, 1]  # set white

    def set_color(self, color):
        self.color = color

    def set_bool_colour(self):
        if self.status:
            return self.set_white()
        else:
            return self.set_red()


class BlueWhiteLedLabel(Label):
    color = ListProperty([1, 1, 1, 1])  # set white as default
    status = BooleanProperty(False)

    def set_blue(self):
        self.color = [0.17, 0.3, 1, 0.5]  # set blue

    def set_white(self):
        self.color = [1, 1, 1, 1]  # set white

    def set_color(self, color):
        self.color = color

    def set_bool_colour(self):
        if self.status:
            return self.set_blue()
        else:
            return self.set_white()


class LedButton(Button):
    color = ListProperty([1, 1, 1, 0.4])  # set grey as default
    status = BooleanProperty(False)

    def set_grey(self):
        self.color = [1, 1, 1, 0.4]  # set grey

    def set_blue(self):
        self.color = [0.17, 0.3, 1, 0.5]  # set blue

    def set_color(self, color):
        self.color = color

    def set_bool_colour(self):
        if self.status:
            return self.set_blue()
        else:
            return self.set_grey()
