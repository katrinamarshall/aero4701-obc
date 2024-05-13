import math
import os
import sys

import kivy
import rospy
from mavros_msgs.msg import AttitudeTarget

kivy.require("1.2.0")
from kivy.app import App
from kivy.clock import Clock
from kivy.graphics import Color, PopMatrix, PushMatrix, Rectangle, Rotate
from kivy.lang import Builder
from kivy.properties import NumericProperty, ObjectProperty, StringProperty
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.widget import Widget
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("imuvisualiserpane.kv"))

# sys.path.append('../../bridge')


class ImuVisualiserPane(RelativeLayout):
    roll_widget = ObjectProperty()
    pitch_widget = ObjectProperty()

    def __init__(self, **kwargs):
        super(ImuVisualiserPane, self).__init__(**kwargs)
        self.pitch_widget.set_label("PITCH")
        self.roll_widget.set_label("ROLL")

        self.odom_sub = rospy.Subscriber("~odometry", Odometry, callback=self.odom_cb)

        print("IMU VIA")

    def odom_cb(self, odom):
        rospy.loginfo("Cb")
        orientation = odom.pose.pose.orientation
        pitch_deg, roll_deg = self.quat_to_pitch_roll(
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z,
        )
        self.pitch_widget.set_angle(round(pitch_deg, 2))
        self.roll_widget.set_angle(round(roll_deg, 2))

    def quat_to_pitch_roll(self, w, x, y, z):
        # Converts a quaternion to pitch and roll
        # roll (y-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = -math.atan2(sinr_cosp, cosr_cosp) * (180 / math.pi)

        # pitch (x-axis rotation)
        sinp = 2 * (w * y - z * x)
        if math.fabs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) * (
                180 / math.pi
            )  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp) * (180 / math.pi)
        return pitch, roll


class PitchRollVisualiser(Widget):
    angle = NumericProperty(0)
    label = StringProperty("")

    def __init__(self, bot_graphic_file, **kwargs):
        super(PitchRollVisualiser, self).__init__(**kwargs)

        # Asset file names
        self.bot_graphic_path = seawolf.form_assets_path(bot_graphic_file)
        self.attitude_graphic_path = seawolf.form_assets_path("attitude-gyro.png")

        # Make it update on resize
        self.bind(pos=self.update_graphic, size=self.update_graphic)

        # Set up the drawing instructions
        with self.canvas:
            self.col = Color(1, 1, 1, 1)
            self.background = Rectangle(
                size=self.size, pos=self.pos, source=self.attitude_graphic_path
            )
            PushMatrix()
            self.rot = Rotate()
            self.rot.angle = self.angle
            self.rot.origin = self.center
            self.bot_img = Rectangle(source=self.bot_graphic_path)
            PopMatrix()

    def set_label(self, label):
        self.label = label

    def set_angle(self, new_angle):
        self.angle = new_angle
        self.update_graphic()

    def update_graphic(self, *args):
        """
        Update the graphic with the new parameters
        """
        self.background.size = self.size
        self.background.pos = self.pos

        self.rot.angle = self.angle
        self.rot.origin = self.center

        # The source image dimensions are 848 x 396
        # Each size dimension is in format A*B*C
        # A = original image dimension
        # B = Normalising in respect to height of pane
        # C = Some constant for scaling
        self.bot_img.size = [
            848 * (self.parent.height / 10000) * 5,
            396 * (self.parent.height / 10000) * 5,
        ]

        # To center on the x axis we:
        #  + the canvas's origin x ordinate
        #  + 1/2 the canvas's width
        #  - 1/2 the src img's width
        self.bot_img.pos = [
            self.x + self.width / 2 - (848 * self.parent.height / 10000 * 5) / 2,
            self.height * 0.6,
        ]


class PitchVisualiser(PitchRollVisualiser):
    bot_graphic_file = "bot_side_view.png"

    def __init__(self, **kwargs):
        super(PitchVisualiser, self).__init__(self.bot_graphic_file, **kwargs)


class RollVisualiser(PitchRollVisualiser):
    bot_graphic_file = "bot_back_view.png"

    def __init__(self, **kwargs):
        super(RollVisualiser, self).__init__(self.bot_graphic_file, **kwargs)


if __name__ == "__main__":

    class WidgetApp(App):
        def build(self):
            return ImuVisualiserPane()

    WidgetApp().run()
