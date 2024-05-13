from __future__ import division

import os
import time
from copy import deepcopy
from enum import Enum
from functools import partial

import cv2
import kivy
import kivy.uix.image as kvimage
import netifaces as ni
import numpy as np
import ros_numpy
import rospy

# compressed images
import sensor_msgs.msg
from kivy.app import App
from kivy.clock import Clock
from kivy.graphics import Color, Rectangle
from kivy.graphics.texture import Texture
from kivy.lang import Builder
from kivy.properties import DictProperty, NumericProperty, ObjectProperty
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.behaviors import ButtonBehavior
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from sensor_msgs.msg import CompressedImage

# uncompressed images
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image as Image_msg


kivy.require("1.2.0")

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("video.kv"))

video_feed_dict = None


class ToggleFlipOptions(Enum):
    HORIZONTAL = (0,)
    VERTICAL = (1,)
    PORT = (2,)
    STBD = (3,)
    RESET = 4


class VideoFeed(object):
    def __init__(self, msg_type):
        self._image_msg = None
        self._image_msg_time = None
        self._sub = None
        self._msg_type = msg_type

    @property
    def frame(self):
        if self.is_frame_available():
            return self.process_image_msg(self._image_msg)
        return None

    @property
    def last_frame_time(self):
        return self._image_msg_time

    def process_image_msg(self, image_msg):
        raise NotImplementedError

    def subscribe(self, topic):
        self._sub = rospy.Subscriber(topic, self._msg_type, self.callback, queue_size=1)

    def unsubscribe(self):
        if self._sub:
            self._sub.unregister()
        self.reset()

    def callback(self, image_msg):
        self._image_msg = image_msg
        self._image_msg_time = rospy.get_time()

    def is_frame_available(self):
        return self._image_msg is not None

    def reset(self):
        self._image_msg = None


class CompressedImageVideoFeed(VideoFeed):
    def __init__(self):
        super(CompressedImageVideoFeed, self).__init__(CompressedImage)

    def process_image_msg(self, image_msg):
        return self.decode_frame(image_msg)

    def decode_frame(self, compressed_im):
        # Comes in as a string
        if compressed_im.data == "":
            return None
        np_arr = np.fromstring(compressed_im.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image_np


class ImageVideoFeed(VideoFeed):
    def __init__(self):
        super(ImageVideoFeed, self).__init__(sensor_msgs.msg.Image)

    def process_image_msg(self, image_msg):
        return self.decode_frame(image_msg)

    def decode_frame(self, image):
        np_arr = ros_numpy.image.image_to_numpy(image)

        channels = len(np_arr.shape)
        if channels != 3:
            # assume mono
            np_arr = cv2.cvtColor(np_arr, cv2.COLOR_GRAY2RGB)
        return np_arr


class VideoFeedFactory(object):
    def __init__(self):
        pass

    def __call__(self, msg_type):
        if msg_type._type == CompressedImage._type:
            return CompressedImageVideoFeed()
        elif msg_type._type == sensor_msgs.msg.Image._type:
            return ImageVideoFeed()
        else:
            rospy.logerr("Unknown video feed type: {}".format(msg_type))


class VideoDisplayer(ButtonBehavior, FloatLayout):
    """
    Terminology:
    display name -> The value shown on the video feed to indicate where the video data is coming from
    video_source (or just source) -> the ros topic which is the image source. If None, indicates no subscription

    Often a mapping of display to topic name is needed


    Args:
        ButtonBehavior (_type_): _description_
        FloatLayout (_type_): _description_
    """

    # dictionary of topics to steam names as loaded unnder the ~video_stream config
    # used to give a topic a more human readable name, if present
    __custom_display_names = None

    config = DictProperty()
    stream = ObjectProperty()
    rotation = NumericProperty(0)

    def __init__(self, **kwargs):
        super(VideoDisplayer, self).__init__(**kwargs)
        VideoDisplayer._set_custom_display_names()
        self.__video_feed_factory = VideoFeedFactory()
        self.__current_source = kwargs.pop("source", None)
        self.__display_name = kwargs.pop("display_name", None)
        self.__msg_type = kwargs.pop("msg_type", None)

        rospy.loginfo("Current source {}".format(self.__current_source))

        if self.__current_source:
            self.__current_source = rospy.resolve_name(self.__current_source)

        rospy.loginfo("Current source {}".format(self.__current_source))
        self.__source_msg_type = None  # the ROS msg type (eiher compressed image or image)
        self.__frame_rate = kwargs.pop("frame_rate", 30.0)
        self.__current_display_name = self._construct_display_name(self.__current_source)
        self.__video_feed = None
        self.__overlay_image = None
        self.__update_stream_timer = None

        # seconds after a fresh frame was recieved before we display an error
        self._display_timeout = 3
        self._corrected_size_hint = 1  # correct the aspect of the image when rotated
        self._flip_vertical = kwargs.pop("flip_vertical", False)
        self._flip_horizontal = kwargs.pop("flip_horizontal", False)

        # on press popup options
        # set size properties
        self._button_width = 120
        self._button_height = 40
        self._popup_width = self._button_width + 40
        # this creates a popup sized based on the number of buttons

        self._update_view(
            self.__display_name,
            self.__current_source,
            self.__msg_type,
        )

    @staticmethod
    def _set_custom_display_names():
        if VideoDisplayer.__custom_display_names is None:
            # reconstruct map but with remapping/resolved namespaces
            feeds = rospy.get_param("~video_feeds")
            resolved_feeds = {}
            for stream_name, unresolved_topic in feeds.items():
                resolved_feeds[rospy.remap_name(unresolved_topic)] = stream_name
            VideoDisplayer.__custom_display_names = resolved_feeds

    def on_press(self):

        display_options = self._get_all_display_options()
        if display_options is None:
            return

        title_txt = "Select feed"
        popup_content = BoxLayout(orientation="vertical")

        for (
            display_name,
            options,
        ) in display_options.items():  # create a button for each video source
            topic = options["topic"]
            msg_type = options["msg_type"]
            option_button = Button(
                size_hint=(None, None),
                size=(self._button_width, self._button_height),
                pos_hint={"center_x": 0.5},
                text=display_name,
                # partial is used to pass args to the switch_feeds function
            )
            option_button.bind(
                on_release=partial(self._update_view, display_name, topic, msg_type)
            )
            # add the buttons to the popup
            popup_content.add_widget(option_button)

        # Buttons responsible for transforming the displayed image
        rotate_button = Button(
            size_hint=(None, None),
            size=(self._button_width, self._button_height),
            pos_hint={"center_x": 0.5},
            text="Rotate 90",
            # partial is used to pass args to the switch_feeds function
            on_release=partial(self.change_rotation, 90),
        )
        popup_content.add_widget(rotate_button)  # add the buttons to the popup

        flip_h_button = Button(
            size_hint=(None, None),
            size=(self._button_width, self._button_height),
            pos_hint={"center_x": 0.5},
            text="Flip H",
            # partial is used to pass args to the switch_feeds function
            on_release=partial(self.toggle_flip, ToggleFlipOptions.HORIZONTAL),
        )
        popup_content.add_widget(flip_h_button)  # add the buttons to the popup

        flip_v_button = Button(
            size_hint=(None, None),
            size=(self._button_width, self._button_height),
            pos_hint={"center_x": 0.5},
            text="Flip V",
            # partial is used to pass args to the switch_feeds function
            on_release=partial(self.toggle_flip, ToggleFlipOptions.VERTICAL),
        )
        popup_content.add_widget(flip_v_button)  # add the buttons to the popup

        port_button = Button(
            size_hint=(None, None),
            size=(self._button_width, self._button_height),
            pos_hint={"center_x": 0.5},
            text="Port Side",
            # partial is used to pass args to the switch_feeds function
            on_release=partial(self.toggle_flip, ToggleFlipOptions.PORT),
        )
        popup_content.add_widget(port_button)  # add the buttons to the popup

        stbd_button = Button(
            size_hint=(None, None),
            size=(self._button_width, self._button_height),
            pos_hint={"center_x": 0.5},
            text="Stbd Side",
            # partial is used to pass args to the switch_feeds function
            on_release=partial(self.toggle_flip, ToggleFlipOptions.STBD),
        )
        popup_content.add_widget(stbd_button)  # add the buttons to the popup

        reset_transform_button = Button(
            size_hint=(None, None),
            size=(self._button_width, self._button_height),
            pos_hint={"center_x": 0.5},
            text="Reset Orient",
            # partial is used to pass args to the switch_feeds function
            on_release=partial(self.toggle_flip, ToggleFlipOptions.RESET),
        )
        popup_content.add_widget(reset_transform_button)  # add the buttons to the popup

        # Create overlay buttons
        cleaning_overlay_button = self._setup_cleaning_overlay_button()
        if cleaning_overlay_button:
            popup_content.add_widget(cleaning_overlay_button)

        grid_overlay_button = self._setup_grid_overlay_button()
        if grid_overlay_button:
            popup_content.add_widget(grid_overlay_button)

        # Aligns buttons to the top of boxlayout
        popup_content.add_widget(BoxLayout())

        visual_spacer = 70

        # Calculate popup height based on popup children - 1, -1 for empty boxlayout that fixes spacing.
        popup_height = (self._button_height * (len(popup_content.children) - 1)) + visual_spacer

        # create popup object
        popup = Popup(
            title=title_txt,
            content=popup_content,
            size_hint=(None, None),
            pos_hint={"center_x": 0.5, "center_y": 0.5},
            size=(self._popup_width, popup_height),
        )
        popup.open()

    def _setup_cleaning_overlay_button(self):
        """Set the cleaning overlay source path and create overlay button if overlay for specific video is set.

        Returns:
            Button: Button to toggle overlay
        """
        overlay_button = None
        button_text = "Cleaning Overlay"

        if self.__display_name == "Left Front":
            self._cleaning_path = seawolf.form_assets_path("left_front_cleaning_overlay.png")
            button_func = partial(self.toggle_overlay, self._cleaning_path)
            overlay_button = self._create_overlay_button(button_text, button_func)
        elif self.__display_name == "Right Front":
            self._cleaning_path = seawolf.form_assets_path("right_front_cleaning_overlay.png")
            button_func = partial(self.toggle_overlay, self._cleaning_path)
            overlay_button = self._create_overlay_button(button_text, button_func)

        return overlay_button

    def _setup_grid_overlay_button(self):
        """Set the grid overlay source path and create overlay button if overlay for specific video is set.

        Returns:
            Button: Button to toggle overlay
        """
        overlay_button = None
        button_text = "Grid Overlay"

        self._grid_path = seawolf.form_assets_path("rule_of_thirds_grid.png")
        button_func = partial(self.toggle_overlay, self._grid_path)
        overlay_button = self._create_overlay_button(button_text, button_func)

        return overlay_button

    def _create_overlay_button(self, button_text, button_func):
        """Create an Button that is binded to toggle overlay.

        Returns:
            Button: Button to toggle overlay
        """
        overlay_button = Button(
            size_hint=(None, None),
            size=(self._button_width, self._button_height),
            pos_hint={"center_x": 0.5},
            text=button_text,
            on_release=button_func,
        )
        return overlay_button

    def _get_all_display_options(self):
        active_topics = self.get_image_topics()

        if active_topics is None:
            rospy.logerr("No active image topics")
            return None

        # display as human readable or not with popup
        # map = {display name : { topic: value, msg_type : value }}
        display_options = {}
        for topic, msg_type in active_topics:
            display_name = self._construct_display_name(topic)
            if display_name is not None:
                display_options[display_name] = {}
                display_options[display_name]["topic"] = topic
                display_options[display_name]["msg_type"] = msg_type
                display_options[display_name]["display_name"] = display_name

        return display_options

    def _get_display_options_by_topic(self, topic):
        default_options = {}
        default_options["topic"] = None
        default_options["msg_type"] = None
        default_options["display_name"] = None

        if topic is None:
            return default_options
        display_options = self._get_all_display_options()

        if display_options is None:
            return default_options

        for _, options in display_options.items():
            if options["topic"] == topic:
                return options
        default_options = {}
        default_options["topic"] = None
        default_options["msg_type"] = None
        default_options["display_name"] = None
        return default_options

    def _update_view(self, display_name, topic, msg_type, *args):
        self.__current_display_name = display_name
        self.__current_source = topic
        self.__source_msg_type = msg_type
        rospy.loginfo(
            "Updating\ndisplay name - {}\nsource - {}".format(
                self.__current_display_name, self.__current_source
            )
        )
        self._unsubscribe()
        self._reset()
        self._subscribe()

        if self.__current_source is None:
            # self._update_view_with_text("No video")
            self._update_view_with_logo()
        else:
            self.clear_widgets()

    def _unsubscribe(self):
        if self.__video_feed:
            self.__video_feed.unsubscribe()

    def _subscribe(self):
        if self.__current_source is None:
            return

        if not self.__video_feed:
            self.__video_feed = self.__video_feed_factory(self.__source_msg_type)
        self.__video_feed.subscribe(self.__current_source)

        frame_duration = 1.0 / self.__frame_rate
        self.__update_stream_timer = Clock.schedule_interval(self._update_stream, frame_duration)

    def _reset(self):
        try:
            # this cancels the previous callback upon stream switch
            self.__update_stream_timer.cancel()
        except:
            pass

    def _construct_display_name(self, source):
        if source is None:
            return "No source selected"
        # check if source has a given stream name
        if source in VideoDisplayer.__custom_display_names:
            return VideoDisplayer.__custom_display_names[source]

        # if no mapping found then dont display the stream
        else:
            return None

    def _update_stream(self, dt):
        if not self.__video_feed:
            self._update_view_with_logo()
            return

        if not self.__video_feed.is_frame_available():
            self._update_view_with_logo()
            return

        now = rospy.get_time()
        last_msg_time = self.__video_feed.last_frame_time

        # check for timeout
        if now - last_msg_time > self._display_timeout:
            self.__video_feed.reset()
            # self._update_view_with_text("No image received")
            self._update_view_with_logo()
            return

        numpy_frame = deepcopy(self.__video_feed.frame)

        if numpy_frame is None:
            return

        self._update_view_with_image(numpy_frame)

        # This is setup code for displaying the video feed that occurs when we get a stream back after it was previously not available or on first start
        # if True or self.vid_displayed == False:
        # Make new widget and add to screen on every iteration, works but slows down dual screen view (when 2 videos streaming)

    def _update_view_with_text(self, msg):
        self.clear_widgets()
        msg_container = AnchorLayout(anchor_x="center", anchor_y="center")
        error_msg = Label(text=msg, size_hint=(None, None), size=(200, 30), color=[1, 0, 0, 1])
        self.add_widget(msg_container)
        msg_container.add_widget(error_msg)

    def _update_view_with_logo(self):
        graphic_path = seawolf.form_assets_path("hullbot_logo_small.png")
        self.clear_widgets()
        self.display_image = kvimage.Image(
            size_hint=(0.15, 0.15),
            pos_hint={"center_x": 0.5, "center_y": 0.5},
            allow_stretch=True,
            source=graphic_path,
        )
        self.add_widget(self.display_image)

    def _update_view_with_image(self, np_image, **kwargs):
        buf = np_image.tostring()
        self.clear_widgets()  # clears the error message
        self.image_ratio = np_image.shape[0] / np_image.shape[1]  # height/width
        self.display_image = kvimage.Image(
            size_hint=(self._corrected_size_hint, 1),
            pos_hint={"center_x": 0.5, "center_y": 0.5},
            allow_stretch=True,
        )
        self.add_widget(self.display_image)
        # this object is used to display the current video frame
        self.texture = Texture.create(size=(np_image.shape[1], np_image.shape[0]), colorfmt="bgr")
        self.vid_displayed = True
        self.error_displayed = False
        # the blit buffer function is a memory efficient way of displaying a new image that may be very similar to the last image
        self.texture.blit_buffer(buf, colorfmt="bgr", bufferfmt="ubyte")
        if self._flip_vertical:
            self.texture.flip_vertical()  # much more efficient than the np equivalent flipud
        if self._flip_horizontal:
            self.texture.flip_horizontal()  # much more efficient than the np equivalent flipud
        # display image from the texture
        self.display_image.texture = self.texture

        if self.__overlay_image:
            self.add_widget(self.__overlay_image)

    def get_image_topics(self):
        """_summary_

        Returns:
            List[Tuples]: List of tuples in the form [topic string, msg type]
        """
        return seawolf.get_topics_by_type(CompressedImage, sensor_msgs.msg.Image)

    def toggle_flip(self, toggle_flip_options, *args):
        """This function sets flags to tell the image texture how to transform

        Args:
            direction (ToggleFlipOptions)
        """
        if toggle_flip_options is ToggleFlipOptions.HORIZONTAL:
            if not self._flip_horizontal:
                self._flip_horizontal = True
            else:
                self._flip_horizontal = False

        elif toggle_flip_options is ToggleFlipOptions.VERTICAL:
            if not self._flip_vertical:
                self._flip_vertical = True
            else:
                self._flip_vertical = False
        elif toggle_flip_options is ToggleFlipOptions.PORT:
            self._flip_horizontal = False
            self._flip_vertical = True
            self.change_rotation(0)
        elif toggle_flip_options is ToggleFlipOptions.STBD:
            self._flip_horizontal = True
            self._flip_vertical = False
            self.change_rotation(0)
        if toggle_flip_options is ToggleFlipOptions.RESET:
            self._flip_vertical = False
            self._flip_horizontal = False
            self.change_rotation(0)

        if self.__overlay_image:
            self.update_overlay(self._flip_horizontal, self._flip_vertical)

    def change_rotation(self, angle, *args):
        """Modifies a variable which is used to rotate the image in the .kv file

        Args:
            angle (float): An increment or decrement for the rotation of the image
        """

        # Check if image ratio attribute is provided by image update updated to prevent seawolf breaking
        if hasattr(self, "image_ratio"):

            if angle == 0:
                self.rotation = 0
                self._corrected_size_hint = 1
            else:
                self.rotation += angle
                # find out if we have a vertical video i.e. 90 or 270 degrees rotation. If vertical we need to adjust the size hint to keep
                # the image inside of its bounding box
                if ((self.rotation / 90) % 2) == 1:
                    # image ratio is the ratio of height to width,
                    self._corrected_size_hint = self.image_ratio
                else:
                    self._corrected_size_hint = 1  # if we are in landscape fill the whole area

            if self.__overlay_image:
                self.update_overlay(self._flip_horizontal, self._flip_vertical)

    def toggle_overlay(self, overlay_path, *args):
        """Toggle overlay image on/off by removing or updating stored overlay."""
        if self.__overlay_image:
            self.__overlay_image = None
        else:
            self._overlay_path = overlay_path
            self.update_overlay(self._flip_horizontal, self._flip_vertical)

    def update_overlay(self, flip_horizontal, flip_vertical):
        """Update and flip overlay image.

        nocache is set to true as any image instace with the same source will have a cached/shared texture.
        This is set to true so flip orientation can be toggled correctly on each updated.
        Overlay opacity can be set via the color property.

        Args:
            flip_horizontal (bool): Flip horizontal
            flip_vertical (bool): Flip vertical
        """
        self.__overlay_image = kvimage.Image(
            size_hint=(self._corrected_size_hint, 1),
            pos_hint={"center_x": 0.5, "center_y": 0.5},
            allow_stretch=True,
            source=self._overlay_path,
            nocache=True,
            color=[1, 1, 1, 0.9],
        )
        if flip_horizontal:
            self.__overlay_image.texture.flip_horizontal()
        if flip_vertical:
            self.__overlay_image.texture.flip_vertical()


if __name__ == "__main__":

    class WidgetApp(App):
        def __init__(self, **kwargs):
            super(WidgetApp, self).__init__(**kwargs)

        def build(self):
            return Video("Mynteye Left")

    rospy.init_node("test_node")
    WidgetApp().run()
