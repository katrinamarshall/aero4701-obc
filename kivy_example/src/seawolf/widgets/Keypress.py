import os
import sys
import traceback
from copy import deepcopy

import kivy
import rospy
from kivy.lang import Builder
from kivy.properties import ListProperty, ObjectProperty
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.widget import Widget
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

kivy.require("1.2.0")

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("Keypress.kv"))


# Variable set by other widgets to disable keystrokes
# Implemented by:
#   MissionNamePopup
KEY_ACTIVE = True
POI_ACTIVE = False


class KeyHandler(Widget):
    """
    Takes keyboard events and maps them to joystick controls, then publishes them on /joy
    Hold shift down to use. Mappings are given in the comments below
    """

    # A value from 0-1 specifying what percentage of full speed to go in any direction with a button press
    increment = 0.9

    # The array indices for controller buttons and axes in the Joy msg
    # Axes
    axes_channels = {"Lx": 0, "Ly": 1, "Rx": 2, "Lt": 3, "Rt": 4, "Ry": 5, "Dx": 6, "Dy": 7}

    # Buttons
    button_channels = {
        "X": 0,
        "A": 1,
        "B": 2,
        "Y": 3,
        "LB": 4,
        "RB": 5,
        "LT": 6,
        "RT": 7,
        "BACK": 8,
        "START": 9,
        "L3": 10,
        "R3": 11,
        "PS": 12,
        "Touchpad": 13,
    }

    # Controller buttons and joysticks
    #          1.0 Y
    #           ^
    #     1.0 < * > -1.0  ---- X
    #           v
    #         -1.0
    # axes: Xl, Yl, Xr, Yr, Xd, Yd
    # l for left joystick, r for right and d for d-pad

    # Keys can be any normal 'unshifted' keyboard character (whatever the key would print without shift pressed)
    # Or it can be any of the strings in the special_chars dictionary
    # Add a 'key_disp' field to the dictionary to change how it will display in KeyboardHelp menu
    # Dir specifies the direction to push the stick (negative is to the right on a stick)
    axes_mapping = {
        "w": {"ax": "Ly", "dir": 1, "control_disp": "Left joystick up"},
        "d": {"ax": "Lx", "dir": -1, "control_disp": "Left joystick right"},
        "s": {"ax": "Ly", "dir": -1, "control_disp": "Left joystick down"},
        "a": {"ax": "Lx", "dir": 1, "control_disp": "Left joystick left"},
        "i": {"ax": "Ry", "dir": 1, "control_disp": "Right joystick up"},
        "l": {"ax": "Rx", "dir": -1, "control_disp": "Right joystick right"},
        "k": {"ax": "Ry", "dir": -1, "control_disp": "Right joystick down"},
        "j": {"ax": "Rx", "dir": 1, "control_disp": "Right joystick left"},
    }

    # X, A, B, Y, LB, RB, LT, RT, BACK, START, L3, R3, Spacebar
    buttons_mapping = {
        "\r": {"but": "START", "key_disp": "[Enter]"},
        "\b": {"but": "BACK", "key_disp": "[Backspace]"},
        "e": {"but": "LB"},
        "c": {"but": "LT"},
        "t": {"but": "L3"},
        "u": {"but": "RB"},
        "n": {"but": "RT"},
        "g": {"but": "R3"},
        "=": {"but": "Y"},
        "]": {"but": "B"},
        "'": {"but": "A"},
        "[": {"but": "X"},
        " ": {"but": "Touchpad"},
    }

    # Dpad maps to the last channels in the axes array
    dpad_mapping = {
        "arr_up": {"ax": "Dy", "dir": 1, "key_disp": "Up arrow", "control_disp": "D-pad up"},
        "arr_right": {
            "ax": "Dx",
            "dir": -1,
            "key_disp": "Right arrow",
            "control_disp": "D-pad right",
        },
        "arr_down": {
            "ax": "Dy",
            "dir": -1,
            "key_disp": "Down arrow",
            "control_disp": "D-pad down",
        },
        "arr_left": {"ax": "Dx", "dir": 1, "key_disp": "Left arrow", "control_disp": "D-pad left"},
    }

    # This can be expanded by printing out the 'ascii' value on key_up or key_down
    special_chars = {
        273: "arr_up",
        274: "arr_down",
        275: "arr_right",
        276: "arr_left",
        280: "pg_up",
        281: "pg_down",
        283: "F2",
    }

    function_mapping = []

    # Add a 'ch' key to store the numerical channel of the controller output being mapped to
    for key in axes_mapping.keys():
        axes_mapping[key]["ch"] = axes_channels[axes_mapping[key]["ax"]]
    for key in dpad_mapping.keys():
        dpad_mapping[key]["ch"] = axes_channels[dpad_mapping[key]["ax"]]
    for key in buttons_mapping.keys():
        buttons_mapping[key]["ch"] = button_channels[buttons_mapping[key]["but"]]

    # Arrays to keep track of button and axis states
    buttons = ListProperty([0] * len(button_channels))
    axes = ListProperty([0.0] * len(axes_channels))

    def __init__(self, **kwargs):
        super(KeyHandler, self).__init__(**kwargs)

        self.pod_motor_estop_pub = rospy.Publisher(
            "/pod_motor_emergency_override", Bool, queue_size=1
        )

        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=1)
        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = "keyboard"

        # Anytime buttons or axes gets updated, pub will automatically be called to
        # publish the new command
        # self.bind(axes=self.pub, buttons=self.pub)
        # Workaround the following assertion error:
        # Traceback (most recent call last):
        #   File "/home/user/catkin_ws/src/seawolf/TopsideApp.py", line 272, in <module>
        #     TopsideApp().run()
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/app.py", line 829, in run
        #     root = self.build()
        #   File "/home/user/catkin_ws/src/seawolf/TopsideApp.py", line 61, in build
        #     seawolf = SeawolfWindow()
        #   File "/home/user/catkin_ws/src/seawolf/TopsideApp.py", line 183, in __init__
        #     self.key_handler = KeyHandler()
        #   File "/home/user/catkin_ws/src/seawolf/widgets/Keypress.py", line 130, in __init__
        #     self.keyboard_help = KeyboardHelp(self)
        #   File "/home/user/catkin_ws/src/seawolf/widgets/Keypress.py", line 199, in __init__
        #     super(KeyboardHelp, self).__init__(**kwargs)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/uix/modalview.py", line 161, in __init__
        #     super(ModalView, self).__init__(**kwargs)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/uix/anchorlayout.py", line 68, in __init__
        #     super(AnchorLayout, self).__init__(**kwargs)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/uix/layout.py", line 76, in __init__
        #     super(Layout, self).__init__(**kwargs)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/uix/widget.py", line 361, in __init__
        #     rule_children=rule_children)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/uix/widget.py", line 469, in apply_class_lang_rules
        #     rule_children=rule_children)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/lang/builder.py", line 538, in apply
        #     rule_children=rule_children)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/lang/builder.py", line 659, in _apply_rule
        #     child, crule, rootrule, rule_children=rule_children)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/lang/builder.py", line 659, in _apply_rule
        #     child, crule, rootrule, rule_children=rule_children)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/lang/builder.py", line 657, in _apply_rule
        #     root=rctx['ids']['root'], rule_children=rule_children)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/uix/widget.py", line 469, in apply_class_lang_rules
        #     rule_children=rule_children)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/lang/builder.py", line 538, in apply
        #     rule_children=rule_children)
        #   File "/usr/local/lib/python2.7/dist-packages/kivy/lang/builder.py", line 554, in _apply_rule
        #     assert(rule not in self.rulectx)
        # AssertionError
        try:
            self.keyboard_help = KeyboardHelp(self)
        except AssertionError as e:
            print("Creating KeyboardHelp created the assertion: '{}'. Retrying.".format(e))
            self.keyboard_help = KeyboardHelp(self)

        self.pub_timer = rospy.Timer(rospy.Duration(1.0 / 20.0), self.pub)

    # Uncomment to see the keycodes of unknown keys
    # def key_down1(self, *args):
    #     print(args)

    def key_down(self, WindowPygameObject, ascii, keycode, text, modifiers):
        # Example arguments:
        # e: (<kivy.core.window.window_pygame.WindowPygame object at 0x7f1c3f08ee50>, 101, 26, u'e', [])
        # CTRL + e: (<kivy.core.window.window_pygame.WindowPygame object at 0x7f1c3f08ee50>, 101, 26, u'\x05', ['ctrl'])
        # Shift: (<kivy.core.window.window_pygame.WindowPygame object at 0x7f1c3f08ee50>, 304, 50, u'', [])

        # 'q' key is used as an emergency stop for pod motors
        if self.ascii_to_key(ascii) == "q":
            self.pod_motor_estop_pub.publish(Bool(True))

        # To open up a window containing all of the key commands that are supported.
        # This window automatically updates to include any new key commands added.
        elif self.ascii_to_key(ascii) == "F2":
            self.keyboard_help.toggle_open()

        # Shift is required for all key controls for bot driving
        elif "shift" in modifiers:
            # KEY_ACTIVE must also be active in case we are regularly typing
            if KEY_ACTIVE:
                key = self.ascii_to_key(ascii)
                if key in self.dpad_mapping:
                    self.axes[self.dpad_mapping[key]["ch"]] = self.dpad_mapping[key]["dir"]
                if key in self.axes_mapping:
                    self.axes[self.axes_mapping[key]["ch"]] = (
                        self.increment * self.axes_mapping[key]["dir"]
                    )
                elif key in self.buttons_mapping:
                    self.buttons[self.buttons_mapping[key]["ch"]] = 1

        # As long as shift isn't pressed, function controls will work
        else:
            in_key = self.ascii_to_key(ascii)
            # Ensure that there are function controls that have been set up
            if len(self.function_mapping) > 0:
                for mapping in self.function_mapping:
                    # This is for closing the POI window, keybound to 'ENTER'
                    if mapping["context"] == "POI" and POI_ACTIVE:
                        if mapping["key"] == in_key:
                            fn_handle = mapping["handle"]
                            fn_handle()
                    # This is for opening the POI window, keybound to 'i'
                    if mapping["context"] == "POI" and not POI_ACTIVE:
                        if mapping["key"] == in_key:
                            fn_handle = mapping["handle"]
                            fn_handle()

    def bind_key_to_func(self, key, fn_handle, context, desc):
        """
        Creates a keybind for a command for a function
        Requires the key needed to map, the function that keypress should execute
        In which command context the key will be pressed in and a description of
        what that command does
        """
        temp_dic = {"key": key, "handle": fn_handle, "context": context, "description": desc}
        self.function_mapping.append(temp_dic)

    def key_up(self, WindowPygameObject, ascii, keycode):
        # Reset the value when the corresponding key is released
        try:
            key = self.ascii_to_key(ascii)
            if key in self.dpad_mapping:
                self.axes[self.dpad_mapping[key]["ch"]] = 0
            elif key in self.axes_mapping:
                self.axes[self.axes_mapping[key]["ch"]] = 0
            elif key in self.buttons_mapping:
                self.buttons[self.buttons_mapping[key]["ch"]] = 0
        except Exception as e:
            traceback.print_exc(file=sys.stdout)

    def ascii_to_key(self, ascii):

        if ascii < 256:
            # Normal ascii characters
            return unichr(ascii)
        elif ascii in self.special_chars:
            # Special keyboard characters not in ascii
            return self.special_chars[ascii]
        else:
            # Undefined
            return None

    def pub(self, *args):
        if self.keyboard_help.keyboard_enable_switch.active:
            self.joy_msg.header.stamp = rospy.Time.now()
            self.joy_msg.axes = deepcopy(self.axes[:])
            self.joy_msg.buttons = deepcopy(self.buttons[:])
            self.joy_pub.publish(self.joy_msg)


class KeyboardHelp(Popup):
    """
    A popup to show the mappings of the keyboard control
    """

    bot_graphic_file = "keyboard_control_inverted.png"

    scroll_content = ObjectProperty(None)
    command_list = ObjectProperty(None)
    keyboard_enable_switch = ObjectProperty(None)

    def __init__(self, key_handler, **kwargs):
        """
        Create the KeyboardHelp popup, implemented with a scrolling list of keys and their mappings
        """

        # Asset file names
        self.bot_graphic_path = seawolf.form_assets_path(self.bot_graphic_file)

        super(KeyboardHelp, self).__init__(**kwargs)
        # self.scroll_content.bind(minimum_height=self.scroll_content.setter('height'))
        self.key_handler = key_handler
        self.is_open = False

        # Calculate the height of the scrolling content after adding all labels
        for child in self.scroll_content.children:
            self.scroll_content.height += child.height

        # Add a label for other keypress controls
        for mapping in key_handler.function_mapping:
            key = mapping["key"]
            desc = mapping["description"]

            if key == "\r":
                key = "ENTER"

            self.command_list.add_widget(Label(text=key))
            self.command_list.add_widget(Label(text=desc))

    def repr(self, key):
        """
        Get the desired human-readable representation of the key using the 'key_disp' and 'control_disp' fields in the mapping
        If the 'key_disp' or 'control_disp' field does not exist, just use the name of the key as is in the mapping dictionary
        """
        key_repr = key
        control_repr = None

        if key in self.key_handler.dpad_mapping:
            if "key_disp" in self.key_handler.dpad_mapping[key]:
                key_repr = self.key_handler.dpad_mapping[key]["key_disp"]

            if "control_disp" in self.key_handler.dpad_mapping[key]:
                control_repr = self.key_handler.dpad_mapping[key]["control_disp"]
            else:
                control_repr = self.key_handler.dpad_mapping[key]["ax"]

        elif key in self.key_handler.axes_mapping:
            if "key_disp" in self.key_handler.axes_mapping[key]:
                key_repr = self.key_handler.axes_mapping[key]["key_disp"]

            if "control_disp" in self.key_handler.axes_mapping[key]:
                control_repr = self.key_handler.axes_mapping[key]["control_disp"]
            else:
                control_repr = self.key_handler.axes_mapping[key]["ax"]

        elif key in self.key_handler.buttons_mapping:
            if "key_disp" in self.key_handler.buttons_mapping[key]:
                key_repr = self.key_handler.buttons_mapping[key]["key_disp"]

            if "control_disp" in self.key_handler.buttons_mapping[key]:
                control_repr = self.key_handler.buttons_mapping[key]["control_disp"]
            else:
                control_repr = self.key_handler.buttons_mapping[key]["but"]
        else:
            raise Exception("Unmapped key passed to KeyboardHelp.repr()")

        return key_repr, control_repr

    def on_open(self):
        self.is_open = True

    def on_dismiss(self):
        self.is_open = False

    def toggle_open(self):
        """
        Toggles popup open/closed
        """
        if self.is_open:
            self.dismiss()
        else:
            self.open()
