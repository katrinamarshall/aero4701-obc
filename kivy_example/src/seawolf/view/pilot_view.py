from kivy.uix.boxlayout import BoxLayout
from sensor_msgs.msg import CompressedImage

from seawolf.widgets.NodeIndicatorBar import NodeIndicatorBar
from seawolf.widgets.Video import VideoDisplayer


class PilotView(BoxLayout):
    def __init__(self, **kwargs):
        super(PilotView, self).__init__(**kwargs)

        self.video_2 = VideoDisplayer(
            source="~video_stream_2",
            display_name="Left Front",
            msg_type=CompressedImage,
            flip_vertical=False,
            flip_horizontal=True,
        )

        self.video_4 = VideoDisplayer(
            source="~video_stream_4", display_name="Left Top", msg_type=CompressedImage
        )

        self.ids.left_video.add_widget(self.video_2)
        self.ids.right_video.add_widget(self.video_4)
        self.ids.node_indicator_bar.add_widget(NodeIndicatorBar())

    def close_video_streams(self):
        self.video_2._unsubscribe()
        self.video_4._unsubscribe()
        self.video_2._reset()
        self.video_4._reset()

    def open_video_streams(self):
        self.video_2._subscribe()
        self.video_4._subscribe()
