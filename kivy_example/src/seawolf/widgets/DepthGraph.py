import copy
from collections import deque

import kivy
import rospy
from kivy.clock import Clock
from nav_msgs.msg import Odometry

import seawolf

kivy.require("1.2.0")
from kivy.app import App
from kivy.garden.graph import Graph, LinePlot
from kivy.lang import Builder
from kivy.properties import ColorProperty, NumericProperty
from kivy.uix.boxlayout import BoxLayout

Builder.load_file(seawolf.form_kivy_config_path("depthGraph.kv"))


class DepthGraph(BoxLayout):
    green = ColorProperty([0, 1, 0, 1])
    red = ColorProperty([0.8, 0, 0, 1])
    depth = NumericProperty(0)

    def __init__(self, **kwargs):
        super(DepthGraph, self).__init__(**kwargs)

        self._timeout = 1
        self._last_time_stamp = 0
        self._odom_sub = rospy.Subscriber("~odometry", Odometry, callback=self._odom_cb)
        self._odom_msg = Odometry()
        self._window_size = 90
        self._depth_deque = deque(maxlen=self._window_size)

        # Create graph
        self._graph_display_range = 1  # m
        self._graph = Graph(
            x_ticks_minor=1,
            y_ticks_minor=5,
            x_ticks_major=self._window_size / 3,
            y_ticks_major=0.5,
            y_grid_label=True,
            x_grid_label=False,
            padding=5,
            x_grid=True,
            y_grid=True,
            xmin=0,
            xmax=self._window_size,
            tick_color=[1, 1, 1, 1],
            ymin=0,
            ymax=self._graph_display_range,
        )

        # Create history plot and horizontal line plot to add to graph
        self._history_plot = LinePlot(color=self.green, line_width=1)
        self._graph.add_plot(self._history_plot)
        self._horizontal_line_plot = LinePlot(color=self.red, line_width=2)
        self._graph.add_plot(self._horizontal_line_plot)
        self.ids.graph.add_widget(self._graph)

        # Schedule graphical updates
        update_rate = 10  # Hz
        Clock.schedule_interval(self._update_graphics_cb, 1.0 / float(update_rate))

    def _odom_cb(self, msg):
        """Store depth in deque and store recieved msg time.

        Args:
            msg (Odometry): msg
        """

        self._last_time_stamp = rospy.get_time()
        self._depth_deque.append(msg.pose.pose.position.z)
        self.depth = round(self._depth_deque[-1], 2)

    def _is_timeout(self):
        """Check timeout between current time and msg recieved time.

        Returns:
            Bool: timeout flag
        """
        now = rospy.get_time()
        return (now - self._last_time_stamp) > self._timeout

    def _update_graphics_cb(self, *args):
        """Update graphics for graph and plots. If timeout, clear depth data."""

        if self._is_timeout():
            self._depth_deque.clear()
            self.depth = 0

        # Deepcopy depth_deque to prevent mutation error while updating graph
        depth_deque = copy.deepcopy(self._depth_deque)
        self._resize_graph_limits(depth_deque)
        self._update_history_line_data(depth_deque)
        self._update_horizontal_line_data(depth_deque)

    def _resize_graph_limits(self, depth_deque):
        """Resizes the y-axis limits of the graph based on current depth."""

        if len(depth_deque) > 1:
            depth = depth_deque[-1]
            previous_depth = depth_deque[-2]

            # If bot depth becomes greater than 0 (bot above water), resize from 0 to graph limit increment
            # i.e  Graph (y-axis min: 0 m | y-axis max: 2 m)
            if depth > 0 and previous_depth < 0:
                self._graph.ymax = self._graph_display_range
                self._graph.ymin = 0

            # If bot depth becomes less than the minimum graph y-axis limit i.e decrement the graph y-axis limits
            # i.e Graph (y-axis min: -2 m | y-axis max: 0 m) ---> Graph (y-axis min: -4 m | y-axis max: -2 m) if graph limit increment is 2 m
            if depth < self._graph.ymin:
                self._graph.ymax = self._graph.ymax - self._graph_display_range
                self._graph.ymin = self._graph.ymin - self._graph_display_range

            # If bot depth becomes greater than the maximum graph y-axis limit i.e increment the graph y-axis limits
            # i.e Graph (y-axis min: -6 m | y-axis max: -4 m) ---> Graph (y-axis min: -4 m | y-axis max: -2 m) if graph limit increment is 2 m
            elif depth > self._graph.ymax:
                self._graph.ymax = self._graph.ymax + self._graph_display_range
                self._graph.ymin = self._graph.ymin + self._graph_display_range

    def _update_history_line_data(self, depth_deque):
        """Update the data of history line plot. If deque is empty, clear data."""

        if depth_deque:
            self._history_plot.points = list(enumerate(depth_deque))
        else:
            self._history_plot.points = []

    def _update_horizontal_line_data(self, depth_deque):
        """Update the data of horizontal line plot. If deque is empty, clear data."""

        if depth_deque:
            self._horizontal_line_plot.points = [
                (0, depth_deque[-1]),
                (self._window_size, depth_deque[-1]),
            ]
        else:
            self._horizontal_line_plot.points = []
