import os
import sys
import unittest

from mock import MagicMock, Mock, patch
from sensor_msgs.msg import CompressedImage, Image

from seawolf.widgets.Video import (
    CompressedImageVideoFeed,
    ImageVideoFeed,
    VideoFeedFactory,
)


# Note: In python2.7 all tests must be called test_*
# Currently unsure how to mock the ExceptionDispatcher and ThreadedExceptionHandler classes to check they
# are called with the correct inputs
class TestVideo(unittest.TestCase):
    def test_video_factory(self):
        video_factory = VideoFeedFactory()

        feed = video_factory(CompressedImage)
        self.assertIsInstance(feed, CompressedImageVideoFeed)

        feed = video_factory(Image)
        self.assertIsInstance(feed, ImageVideoFeed)
