from kivy.uix.boxlayout import BoxLayout
from sensor_msgs.msg import CompressedImage

from seawolf.widgets.Video import VideoDisplayer


class DemoView(BoxLayout):
    def __init__(self, **kwargs):
        super(DemoView, self).__init__(**kwargs)

        # Video container setup
        self.video_0 = VideoDisplayer(
            source="~video_stream_0",
            display_name="Right Top",
            msg_type=CompressedImage,
            flip_horizontal=True,
        )
        self.video_1 = VideoDisplayer(
            source="~video_stream_1",
            display_name="Right Side",
            msg_type=CompressedImage,
            flip_vertical=True,
        )
        self.video_2 = VideoDisplayer(
            source="~video_stream_2",
            display_name="Left Front",
            msg_type=CompressedImage,
            flip_vertical=True,
        )
        self.video_3 = VideoDisplayer(
            source="~video_stream_3",
            display_name="Right Front",
            msg_type=CompressedImage,
            flip_vertical=True,
        )
        self.video_4 = VideoDisplayer(
            source="~video_stream_4",
            display_name="Left Top",
            msg_type=CompressedImage,
            flip_horizontal=True,
        )
        self.video_5 = VideoDisplayer(
            source="~video_stream_5",
            display_name="Left Side",
            msg_type=CompressedImage,
            flip_vertical=True,
        )
        self.ids.right_side_video.add_widget(self.video_1)
        self.ids.right_front_video.add_widget(self.video_3)
        self.ids.right_top_video.add_widget(self.video_0)
        self.ids.left_side_video.add_widget(self.video_5)
        self.ids.left_front_video.add_widget(self.video_2)
        self.ids.left_top_video.add_widget(self.video_4)

    def close_video_streams(self):
        self.video_0._unsubscribe()
        self.video_1._unsubscribe()
        self.video_2._unsubscribe()
        self.video_3._unsubscribe()
        self.video_4._unsubscribe()
        self.video_5._unsubscribe()

        self.video_0._reset()
        self.video_1._reset()
        self.video_2._reset()
        self.video_3._reset()
        self.video_4._reset()
        self.video_5._reset()

    def open_video_streams(self):
        self.video_0._subscribe()
        self.video_1._subscribe()
        self.video_2._subscribe()
        self.video_3._subscribe()
        self.video_4._subscribe()
        self.video_5._subscribe()
