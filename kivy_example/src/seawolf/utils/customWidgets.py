from tokenize import String

import kivy

kivy.require("1.2.0")
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.properties import ListProperty, NumericProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.uix.textinput import TextInput

import seawolf

Builder.load_string(
    """
<CustomIndicator>:
    canvas.before:
        Color:
            rgba: root.background_color
        Rectangle:
            size: self.size
            pos: self.pos

    text: root.label

<CustomLabelledIndicator>:
    canvas.before:
        Color:
            rgba: root.background_color
        Rectangle:
            size: self.size
            pos: self.pos

    BoxLayout:
        orientation: 'vertical'
        Label:
            size_hint: (1,0.4)
            text: root.title
            color: root.text_color
            font_size: root.font_size * 0.75
            font_name: "formular"
        Label:
            size_hint: (1,0.6)
            text: root.label
            color: root.text_color
            font_size: root.font_size
            font_name: "formular"

<CustomTextIndicator>:
    size_hint: (1, 1)
    font_size: self.height * 0.45 if self.height * 0.45 < self.width / 10 else self.width / 10
    font_name: "formular"

<CustomSliderIndicator>:
    # https://github.com/kivy/kivy/blob/master/kivy/data/style.kv
    canvas:
        Color:
            rgb: 1, 1, 1
        BorderImage:
            border: self.border_horizontal if self.orientation == 'horizontal' else self.border_vertical
            pos: (self.x + self.padding, self.center_y - self.background_width / 2) if self.orientation == 'horizontal' else (self.center_x - self.background_width / 2, self.y + self.padding)
            size: (self.width - self.padding * 2, self.background_width) if self.orientation == 'horizontal' else (self.background_width, self.height - self.padding * 2)
            source: self.background_horizontal if self.orientation == 'horizontal' else self.background_vertical
    cursor_image: root.custom_cursor_image
    cursor_disabled_image: root.custom_cursor_image
    disabled: True
    background_width: 20

<CustomButton>
    font_name: "formular"

<CustomButtonModern>
    color: 1, 1, 1, 1
    canvas.before:
        Color:
            rgba: root.border
        Rectangle:
            size: self.size
            pos: self.pos

"""
)


class CustomButton(Button):
    # Custom properties here
    color = ListProperty([1, 1, 1, 1])
    background_color = ListProperty([1, 1, 1, 1])
    size_hint_max_y = NumericProperty(60)
    padding_x = NumericProperty()
    text = StringProperty("")

    def __init__(self, **kwargs):
        Button.__init__(self, **kwargs)
        # self.size_hint_min_y = self.texture_size[1]

    def set_text_color(self, red=1, green=1, blue=1, intensity=1):
        # Sets to defined colour
        self.color = [red, green, blue, intensity]

    def set_background_color(self, red=1, green=1, blue=1, intensity=1):
        # Sets to defined colour
        self.background_color = [red, green, blue, intensity]

    def set_true(self):
        # Sets to Green
        self.background_color = [0, 1, 0, 1]

    def set_warn(self):
        # Sets to Yellow
        self.background_color = [1, 1, 0, 1]

    def set_false(self):
        # Sets to Red
        self.background_color = [1, 0, 0, 1]

    def set_neutral(self):
        # Sets to Gray
        self.background_color = [0.5, 0.5, 0.5, 1]

    def set_black(self):
        # Sets to Black
        self.background_color = [0, 0, 0, 1]

    def set_state(self, state):
        if state:
            self.set_true()
        else:
            self.set_false()

    def set_label(self, new_text):
        self.text = new_text.upper()


class CustomButtonModern(CustomButton):
    background_color = ListProperty([0, 0, 0, 1])

    def __init__(self, **kwargs):
        CustomButton.__init__(self, **kwargs)

    def set_black(self, *args):
        self.background_color = [0, 0, 0, 1]

    def set_blue(self, *args):
        self.background_color = [0, 0, 0.5, 1]

    def on_press_cb(self):
        self.set_blue()
        Clock.schedule_once(self.set_black, 1)


class CustomSlider(Slider):
    pass


class CustomLabel(Label):
    text = StringProperty("")


class CustomIndicator(Label):
    label = StringProperty("")
    color = ListProperty([1, 1, 1, 1])  # Default to white
    background_color = ListProperty([0.5, 0.5, 0.5, 1])  # Default to grey
    size_hint_max_y = NumericProperty(60)

    def set_text_color(self, red=1, green=1, blue=1, intensity=1):
        # Sets to defined colour
        self.color = [red, green, blue, intensity]

    def set_background_color(self, red=1, green=1, blue=1, intensity=1):
        # Sets to defined colour
        self.background_color = [red, green, blue, intensity]

    def set_good(self):
        # Sets to Green
        self.background_color = [0, 1, 0, 1]
        self.color = [0, 0, 0, 0]

    def set_bad(self):
        # Sets to Red
        self.background_color = [1, 0, 0, 1]
        self.color = [1, 1, 1, 1]

    def set_warn(self):
        # Sets to Orange
        self.background_color = [1, 1, 0, 1]
        self.color = [0, 0, 0, 0]

    def set_on(self):
        # Sets to Blue
        self.background_color = [0, 0, 1, 1]
        self.color = [0, 0, 0, 1]

    def set_neutral(self):
        # Sets to Gray
        self.background_color = [0.5, 0.5, 0.5, 1]
        self.color = [1, 1, 1, 1]


class CustomLabelledIndicator(BoxLayout):
    title = StringProperty("")
    label = StringProperty("")
    text_color = ListProperty([1, 1, 1, 1])  # Default to white
    background_color = ListProperty([0.5, 0.5, 0.5, 1])  # Default to grey
    size_hint_max_y = NumericProperty(80)
    font_size = NumericProperty()

    def set_text_color(self, red=1, green=1, blue=1, intensity=1):
        # Sets to defined colour
        self.text_color = [red, green, blue, intensity]

    def set_background_color(self, red=1, green=1, blue=1, intensity=1):
        # Sets to defined colour
        self.background_color = [red, green, blue, intensity]

    def set_good(self):
        # Sets to Green
        self.background_color = [0, 1, 0, 1]
        self.text_color = [0, 0, 0, 0]

    def set_bad(self):
        # Sets to Red
        self.background_color = [1, 0, 0, 1]
        self.text_color = [1, 1, 1, 1]

    def set_warn(self):
        # Sets to Orange
        self.background_color = [1, 1, 0, 1]
        self.text_color = [0, 0, 0, 0]


class CustomTextIndicator(Label):
    color = ListProperty([1, 1, 1, 1])  # Default to white
    text = StringProperty("")

    def set_success(self):
        # Sets to Green
        self.color = [0, 1, 0, 1]

    def set_warn(self):
        # Sets to Orange
        self.color = [1, 1, 0, 1]

    def set_error(self):
        # Sets to Red
        self.color = [1, 0, 0, 1]

    def set_neutral(self):
        # Sets to White
        self.color = [1, 1, 1, 1]

    def set_unresponsive(self):
        # Sets to Gray
        self.color = [0.5, 0.5, 0.5, 1]

    def set_text_colour(self, rgba):
        # Sets to color property object
        self.color = rgba

    def set_text(self, newLabel):
        self.text = newLabel.upper()


class CustomSliderIndicator(Slider):
    def __init__(self, **kwargs):
        self.custom_cursor_image = seawolf.form_assets_path("circle.png")
        super(CustomSliderIndicator, self).__init__(**kwargs)


class PositiveTextInput(TextInput):
    """make sure to set input_filter to 'float' or 'int' for numerical input

    Args:
        TextInput (_type_): _description_
    """

    def insert_text(self, substring, from_undo=False):
        return super(PositiveTextInput, self).insert_text(
            substring.replace("-", ""), from_undo=from_undo
        )
