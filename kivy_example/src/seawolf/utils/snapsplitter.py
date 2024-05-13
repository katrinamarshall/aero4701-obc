import kivy

kivy.require("1.2.0")
from kivy.clock import Clock
from kivy.core.window import Window
from kivy.lang import Builder
from kivy.properties import NumericProperty
from kivy.uix.splitter import Splitter, SplitterStrip

Builder.load_string(
    """
<CustomSplitterStrip>:
    canvas:
        Color:
            rgba: 1, 1, 1, 1
        Rectangle:
            size: self.size
            pos: self.pos

<SnapSplitter>:
    min_size: 5
    max_size: 350
    strip_size: 5
    strip_cls: root.strip_cls
    keep_within_parent: True
    rescale_with_parent: False

"""
)


class CustomSplitterStrip(SplitterStrip):
    def __init__(self, **kwargs):
        super(CustomSplitterStrip, self).__init__(**kwargs)


class SnapSplitter(Splitter):
    snap_point = NumericProperty(250)
    strip_cls = CustomSplitterStrip

    def __init__(self, **kwargs):
        super(SnapSplitter, self).__init__(**kwargs)
        Clock.schedule_once(self.hide)

    # To hide the splitters at launch we need to set the size to the minimum
    # in the relevant dimension, however this messes with the splitter scaling
    # behaviour so we size_hint back to None immediately.
    def hide(self, dt):
        if self.sizable_from[0] in ("t", "b"):
            self.size[1] = self.min_size
            self.size_hint_y = None
            # A delay is required so the resizing done in the hide function does
            # not trigger the snap behaviour
            Clock.schedule_once(lambda _: self.bind(height=self.snap))
        else:
            self.size[0] = self.min_size
            self.size_hint_x = None
            Clock.schedule_once(lambda _: self.bind(width=self.snap))

    def snap(self, instance, size):
        if self == instance:
            # Determine if we're working in X or Y
            if self.sizable_from in ("top", "bottom"):
                dim = 1
            else:
                dim = 0
            # Set relevant point of reference
            if self.sizable_from in ("top", "right"):
                mouse_dist = abs(int(self.pos[dim]) - Window.mouse_pos[dim])
            else:
                mouse_dist = abs(int(Window.size[dim]) - Window.mouse_pos[dim])
            # Snap behaviour
            if mouse_dist > self.snap_point * 0.3:
                self.size[dim] = min(max(self.snap_point, mouse_dist), self.max_size)
            else:
                self.size[dim] = self.min_size
