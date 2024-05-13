import os

import kivy

kivy.require("1.2.0")
from diagnostic_msgs.msg import DiagnosticArray
from kivy.app import App
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.properties import DictProperty, ListProperty, ObjectProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from std_msgs.msg import Int8
from xmlrpclib_timeout import ServerProxy

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("nodeStatusPane.kv"))


PROCESS = 0
SIDE = 1
NODE = 2


class CustomDropDown(DropDown):

    node_name = StringProperty("")
    server = ObjectProperty()
    node_dict = DictProperty()

    def __init__(self, **kwargs):
        super(CustomDropDown, self).__init__(**kwargs)

    def start(self):
        process_name = (self.node_dict[self.node_name])[PROCESS]

        # This does not give me joy but it is to account for joy since it does not have a process name
        # that we can restart on supervisor
        if process_name != "":
            try:
                result = self.server.supervisor.startProcess(process_name)
            except Exception as e:
                rospy.logerr("Failed to start node")

    def stop(self):
        process_name = (self.node_dict[self.node_name])[PROCESS]

        if process_name != "":
            try:
                result = self.server.supervisor.stopProcess(process_name)
            except Exception as e:
                rospy.logerr("Failed to stop node")

    def restart(self):
        self.stop()
        self.start()


class NodeIndicator(BoxLayout):
    statusColour = ListProperty([1, 1, 1, 0])  # set grey by default
    label = StringProperty("")  # the label on the indicator
    node_name = StringProperty("")  # the name of the node
    server = ObjectProperty()
    node_dict = DictProperty()

    def __init__(self, **kwargs):
        super(NodeIndicator, self).__init__(**kwargs)

        # Add a button to the top pane
        self.dropdown_container = self.ids.dropdown_menu
        self.mainbutton = Button(text=self.label)
        self.dropdown_container.add_widget(self.mainbutton)

        # For actual nodes, add a drop down menu
        if self.node_name != "NOT_A_NODE" and self.label != "Joy":
            self.dropdown = CustomDropDown(
                node_name=self.label, server=self.server, node_dict=self.node_dict
            )
            self.mainbutton.bind(on_release=self.dropdown.open)

    def getNodeName(self):
        return self.node_name

    def getNodeLabel(self):
        return self.label

    def alive(self):
        self.mainbutton.background_color = [0.17, 0.3, 1, 1]  # Blue

    def dead(self):
        self.mainbutton.background_color = [1, 1, 1, 1]  # Grey

    def warning(self):
        self.mainbutton.background_color = [1, 0.6, 0, 1]  # Yellow

    def error(self):
        self.mainbutton.background_color = [1, 0, 0, 1]  # Red


class NodeStatusPane(BoxLayout):
    # initialise all node indicators

    def __init__(self, **kwargs):
        super(NodeStatusPane, self).__init__(**kwargs)
        NodeIndicatorGroup.get_instance(self.ids.indicator_display)


class NodeIndicatorGroup:
    # initialise all node indicators
    instance = None
    node_list = []
    systemTestInds = []
    min_state = False

    @staticmethod
    def get_instance(indicator_container):
        if NodeIndicatorGroup.instance is None:
            NodeIndicatorGroup.instance = NodeIndicatorGroup()
        NodeIndicatorGroup.instance.add_container(indicator_container)
        return NodeIndicatorGroup.instance

    def __init__(self, **kwargs):
        # Node list
        self.node_names = [
            "/manual_node",
            "/joy_node",
            "/state_machine_node",
            "/thruster_control_node",
            "/cleaning_cores_node",
            "/best_hold_node",
            "/tofs_node",
            "/lights_node",
            "/front_cam_gscam",
            "/trajectory_node",
            "/motion_controller_node",
            "/control_mode_handler_node",
            "/moon_landing_node",
            "/rovio",
            "/localisation/ekf_odom",
            "/orbslam_mono_odom",
            "/bag_rec",
            "/depth_prediction_node",
        ]
        self.node_labels = [
            "Manual",
            "Joy",
            "State Machine",
            "Thruster Ctrl",
            "CCs",
            "Best Plane",
            "TOFs",
            "Lights",
            "Front Cam",
            "Trajectory",
            "6DOF",
            "Control Modes",
            "Moon Land",
            "Rovio",
            "Odometry",
            "SLAM",
            "Rosbag",
            "Depth Est.",
        ]

        # This is a list of nodes that should be ignored in the calculation of the overall bot state (min state) based on features that do not exist on all bots
        # Once a feature exists throughout the fleet, it can be removed
        self.node_ignore = ["Depth Est."]

        # Dictionary from node name to process name on supervisor, which side they are on (topside, botside, podside) and the actual node object
        self.node_dict = {}
        self.node_dict = dict.fromkeys(self.node_labels)
        self.node_dict["Manual"] = ["topside_core:manual_node", "topside"]
        self.node_dict["Joy"] = ["", "topside"]
        self.node_dict["State Machine"] = ["topside_core:state_machine_node", "topside"]
        self.node_dict["Thruster Ctrl"] = [
            "run_common:hb_control_thruster_control",
            "botside",
        ]
        self.node_dict["CCs"] = ["run_common:hb_drivers_cleaning_cores", "botside"]
        self.node_dict["Best Plane"] = [
            "run_common:hb_perception_best_plane_filter",
            "botside",
        ]
        self.node_dict["TOFs"] = ["run_common:hb_drivers_tofs", "botside"]
        self.node_dict["Lights"] = ["run_common:hb_drivers_lights", "botside"]
        self.node_dict["Front Cam"] = ["run_common:front_cam", "botside"]
        self.node_dict["Trajectory"] = [
            "run_common:hb_navigation_trajectory",
            "botside",
        ]
        self.node_dict["6DOF"] = ["run_common:hb_control_motion_controller", "botside"]
        self.node_dict["Control Modes"] = [
            "run_common:hb_control_control_mode",
            "botside",
        ]
        self.node_dict["Moon Land"] = ["run_common:hb_control_moon_landing", "botside"]
        self.node_dict["Rovio"] = ["rovio", "botside"]
        self.node_dict["Odometry"] = [
            "run_common:hb_localisation_core_ekf_odom",
            "botside",
        ]
        self.node_dict["SLAM"] = ["slam:start_slam", "topside"]
        self.node_dict["Rosbag"] = ["rosbag_record", "topside"]
        self.node_dict["Depth Est."] = ["depth_prediction", "topside"]

        # Systest Status
        rospy.Subscriber("/hullbot/system_test_status", Int8, callback=self.system_test_cb)
        self.system_test_status = -1

        # Subscribe to the latest set of diagnostics
        self.aggDiagnosticsSub = rospy.Subscriber(
            "/diagnostics_agg", DiagnosticArray, self.aggDiagnosticsCB
        )

        # Diagnostic level messages
        self.OK = 0
        self.WARN = 1
        self.ERROR = 2
        self.STALE = 3

        # Supervisor client for topside_core
        try:
            self.topside_proc_server = ServerProxy("http://192.168.2.1:9001", timeout=5.0)
        except Exception as e:
            print("Exception when creating ServerProxy to topside: {}".format(e))

        # Supervisor client for tx2_core
        try:
            self.bot_proc_server = ServerProxy("http://192.168.2.2:9001", timeout=5.0)
        except Exception as e:
            print("Exception when creating ServerProxy to botside: {}".format(e))

        # Callback for updating the status of these indicators
        Clock.schedule_interval(self.nodeStatusCB, 0.25)

    def add_container(self, indicator_container):
        # Create Nodes
        for index, _ in enumerate(self.node_labels):
            # Differentiate nodes based on whether they are on topside or botside. Can add podside in future if needed
            if (self.node_dict[self.node_labels[index]])[SIDE] == "topside":
                myNodeInd = NodeIndicator(
                    node_name=self.node_names[index],
                    label=self.node_labels[index],
                    server=self.topside_proc_server,
                    node_dict=self.node_dict,
                )

            elif (self.node_dict[self.node_labels[index]])[SIDE] == "botside":
                myNodeInd = NodeIndicator(
                    node_name=self.node_names[index],
                    label=self.node_labels[index],
                    server=self.bot_proc_server,
                    node_dict=self.node_dict,
                )

            # Create a widget for each node
            indicator_container.add_widget(myNodeInd)
            (self.node_dict[self.node_labels[index]]).append(myNodeInd)

            if self.node_labels[index] not in self.node_ignore:
                self.node_list.append(myNodeInd)

        # Reuse NodeIndicator for SystemTest status
        systemTestInd = NodeIndicator(label="System Test", node_name="NOT_A_NODE")
        systemTestInd.dead()
        self.systemTestInds.append(systemTestInd)
        indicator_container.add_widget(systemTestInd)

    def aggDiagnosticsCB(self, msg):
        statuses = msg.status

        # Go through all messages
        for message in statuses:

            # Strip the /
            name = message.name
            name = name[1:]

            # Use node name to get the actual node object and change its color based on status
            if name in self.node_dict:
                for idx in range(NODE, len(self.node_dict[name])):
                    node = (self.node_dict[name])[idx]

                    if message.level == self.OK:
                        node.alive()
                    elif message.level == self.WARN:
                        node.warning()
                    elif message.level == self.ERROR:
                        node.error()
                    elif message.level == self.STALE:
                        node.dead()
                    else:
                        print("Unexpected behaviour with {}".format(name))

    # Checks the diagnostic status of each node
    def nodeStatusCB(self, dt):
        if not rospy.is_shutdown():
            alive_nodes = rosnode.get_node_names()

            # Check if any nodes are dead
            self.min_state = True
            for node in self.node_list:
                if node.getNodeName() not in alive_nodes:
                    self.min_state = False

            # Special case for joy since it is a ROS node
            for idx in range(NODE, len(self.node_dict["Joy"])):
                if "/joy_node" in alive_nodes:
                    (self.node_dict["Joy"])[idx].alive()
                else:
                    (self.node_dict["Joy"])[idx].dead()

            # System Test results
            for systestInd in self.systemTestInds:
                if self.system_test_status == self.OK:
                    systestInd.alive()
                elif self.system_test_status == self.WARN:
                    systestInd.warning()
                elif self.system_test_status == self.ERROR:
                    systestInd.error()
        else:
            print("Rospy is shutdown!")

    def system_test_cb(self, status):
        self.system_test_status = status.data

    def get_status(self):
        return self.min_state


if __name__ == "__main__":  # For testing widget in isolation

    class WidgetApp(App):
        def build(self):
            return NodeStatusPane()

    class rosnode:  # Dummy data and rosnode class
        def get_node_names(self):
            return ["/manual_node", "/joy_node", "/user_video1_node"]

    rosnode = rosnode()
    WidgetApp().run()

else:  # If running as part of TopsideApp, import the real rosnode library
    import rosnode
    import rospy
