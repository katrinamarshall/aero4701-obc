import math
from enum import Enum

import kivy
import rospy

kivy.require("1.2.0")
from dismissable_popup import show_dismissable_popup
from hullbot_common.quat_calc import quat_to_eul_deg
from kivy.app import App
from kivy.properties import ListProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.popup import Popup
from mavros_msgs.msg import VFR_HUD, AttitudeTarget, State
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Bool, String

from seawolf.utils.ThreadingSupport import threaded


class directions(Enum):
    PORT = 1
    STBD = 2
    BOW = 3
    STERN = 4


class RunBuoyancyTest(BoxLayout):
    statusColour = ListProperty([1, 1, 1, 0.4])  # set grey by default
    label = StringProperty("")  # the label on the indicator

    def __init__(self, **kwargs):
        super(RunBuoyancyTest, self).__init__(**kwargs)


class BuoyancyTestPane(BoxLayout):
    def __init__(self, **kwargs):
        super(BuoyancyTestPane, self).__init__(**kwargs)
        self.indicator_container = self.ids.indicator_display
        self.now_testing = 0
        self.is_underwater = False
        self.armed = False
        self.control_mode = ""

        rospy.Subscriber("/Hullbot/bridge/heartbeat/state", State, self.state_cb)
        rospy.Subscriber(
            "/Hullbot/bridge/mav_sensor/underwater", Bool, callback=self.underwater_cb
        )
        rospy.Subscriber(
            "/hullbot/control_mode_handler/control_mode", String, callback=self.control_mode_cb
        )

        rospy.Subscriber("/Hullbot/bridge/mav_sensor/vfr_hud", VFR_HUD, self.baro_cb)
        rospy.Subscriber("/Hullbot/bridge/mav_sensor/attitude", AttitudeTarget, self.pose_cb)
        self.baro_data = {"data": [], "times": []}
        self.pose_data = {"data": {"roll": [], "pitch": []}, "times": []}
        self.start_time = None
        self.start_depth = None

        self.arm_service = rospy.ServiceProxy("/Hullbot/bridge/mav_srv/arming", CommandBool)

        self.run_buoyancy_test_button = RunBuoyancyTest(size_hint=(1, 1))
        self.run_buoyancy_test_button.ids.run_buoyancy_test_button.bind(
            on_release=self.run_buoyancy_test
        )
        self.run_buoyancy_test_button.label = "Run Buoyancy Test"
        self.run_buoyancy_test_button.ids.run_buoyancy_test_button.disabled = False
        self.indicator_container.add_widget(self.run_buoyancy_test_button)

        # Maximum change in depth
        self.depth_lims = 0.5

        self.running = False
        self.collect_data = False

        self.popup = Popup()

    def state_cb(self, state):
        """
        Subscriber to detect if the bot is armed
        """
        self.armed = state.armed

    def underwater_cb(self, is_underwater):
        """
        Subscriber to detect if the bot is underwater
        """
        self.is_underwater = is_underwater.data

    def control_mode_cb(self, mode):
        """
        Check the current control mode of the bot
        """
        self.control_mode = mode.data

    def baro_cb(self, msg):
        """
        Get depth data
        """
        if self.collect_data:
            # rospy.loginfo(len(self.baro_data['data']))
            if self.start_depth is None:
                self.start_depth = msg.altitude
            self.baro_data["data"].append(msg.altitude - self.start_depth)

            if self.start_time is None:
                self.start_time = msg.header.stamp.to_sec()
            self.baro_data["times"].append(msg.header.stamp.to_sec() - self.start_time)

    def pose_cb(self, msg):
        """
        Get pitch/roll data
        """
        if self.collect_data:
            [roll_deg, pitch_deg, yaw_deg] = quat_to_eul_deg(msg.orientation, return_type=list)
            self.pose_data["data"]["roll"].append(roll_deg)
            self.pose_data["data"]["pitch"].append(-pitch_deg)

            if self.start_time is None:
                self.pose_data["times"].append(0)
                self.start_time = msg.header.stamp.to_sec()
            else:
                self.pose_data["times"].append(msg.header.stamp.to_sec() - self.start_time)

    def check_bot_upright(self):
        """
        Check if bot is orientated correctly prior to starting test
        """
        try:
            attitude = rospy.wait_for_message(
                "/Hullbot/bridge/mav_sensor/attitude", AttitudeTarget, timeout=5
            )
        except (rospy.ROSInterruptException, rospy.ROSException) as e:
            rospy.logerr("Waiting for AttitudeTarget message failed: %s" % (e,))
            return False

        angle_tolerance = 10  # degree tolerance from upright position (0 deg)
        current_eul = quat_to_eul_deg(attitude.orientation)
        roll_valid = current_eul.x > -angle_tolerance and current_eul.x < angle_tolerance
        pitch_valid = current_eul.y > -angle_tolerance and current_eul.y < angle_tolerance

        if roll_valid and pitch_valid:
            return True
        else:
            return False

    @threaded
    def start_test(self, *args):
        """
        Executes buoyancy test by killing all TCs (disarm bot) and tracking robot pose
        """
        self.popup.dismiss()

        # Reset the variables in case of running multiple times
        self.baro_data = {"data": [], "times": []}
        self.pose_data = {"data": {"roll": [], "pitch": []}, "times": []}
        self.start_time = None
        self.start_depth = None

        # Bot must be running underwater in depth hold mode
        if not (self.armed and self.is_underwater and self.control_mode == "DEPTH_HOLD"):
            popup_text = "Bot must be hovering underwater before starting test"
            show_dismissable_popup(title="POSITION BOT", text=popup_text, size_x=400, size_y=300)
            return

        # Bot must be upright (mynteye up and level)
        if not self.check_bot_upright():
            popup_text = (
                "Bot must be oriented flat with mynteye cameras facing up before starting test"
            )
            show_dismissable_popup(
                title="CHECK BOT ORIENTATION", text=popup_text, size_x=600, size_y=300
            )
            return

        self.running = True
        self.run_buoyancy_test_button.label = "Stop Buoyancy Test"

        # Disarm robot to fall under its own motion
        try:
            self.arm_service(False)
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to disarm bot for buoyancy test: " + str(e))

        # Delay to allow effect of thrusters to stop
        rospy.sleep(0.15)

        # Start test (callbacks collect information)
        self.collect_data = True
        rospy.sleep(2)
        self.collect_data = False

        self.end_buoyancy_test()

    def display_results(self):
        """
        Process results and display them in a popup window
        """
        # Continue until one of the units hits angle limit degrees or end (whichever is sooner)
        roll_limit = 75
        pitch_limit = 50
        idx = 0
        for el in self.pose_data["times"]:
            if (
                self.pose_data["data"]["roll"][idx] > roll_limit
                or self.pose_data["data"]["roll"][idx] < -roll_limit
            ):
                break

            if (
                self.pose_data["data"]["pitch"][idx] > pitch_limit
                or self.pose_data["data"]["pitch"][idx] < -pitch_limit
            ):
                break

            idx += 1

        # idx is the position that should be taken
        rerun = False
        if idx == len(self.pose_data["times"]):
            # The last data point is the time reached in the whole two second period
            roll_end = self.pose_data["data"]["roll"][-1]
            pitch_end = self.pose_data["data"]["pitch"][-1]
        else:
            # Find roll/pitch at end point
            roll_end = self.pose_data["data"]["roll"][idx]
            pitch_end = self.pose_data["data"]["pitch"][idx]
            # Recommend changing and running again if reached the edge of boundaries
            rerun = True

        # Calculate primary rise/pitch/roll and required units of weight
        rise = (self.baro_data["data"][-1] - self.baro_data["data"][0]) * 1000
        roll = roll_end - self.pose_data["data"]["roll"][0]
        pitch = pitch_end - self.pose_data["data"]["pitch"][0]

        add_units = self.find_mass_add(rise)
        side_units, side_add, end_units, end_add = self.find_mass_dist(roll, pitch)

        weights = [0] * 4
        locations = ["port forward", "stbd forward", "port back", "stbd back"]

        # Look at compartment opposite two locations to add (side_add and end_add)
        # and remove minimum of two adding amounts (side_units, end_units)
        remove_opposite = min(side_units, end_units)
        remove_id = 0

        if side_add is directions.PORT:
            remove_id += 1
        if end_add is directions.BOW:
            remove_id += 2
        weights[remove_id] -= remove_opposite

        if remove_opposite == side_units and remove_opposite == end_units:
            # Add no more
            pass
        elif remove_opposite == side_units:
            # Add more to end
            # Find the closest even (round up)
            final_add = math.ceil((end_units - remove_opposite) / 2)
            add_id = 0 if end_add is directions.BOW else 2

            weights[add_id] += final_add
            weights[add_id + 1] += final_add

        else:
            # Add more to side
            # Find the closest even (round up)
            final_add = math.ceil((side_units - remove_opposite) / 2)
            add_id = 0 if side_add is directions.PORT else 1

            weights[add_id] += final_add
            weights[add_id + 2] += final_add

        # Calculate the total motion based on rise/fall
        total_added = sum(weights)
        to_add = math.floor((add_units - total_added) / 4)
        for id in range(4):
            weights[id] += to_add

        # Display results
        results_text = "Buoyancy Test Results"
        for idx, el in enumerate(weights):
            if el >= 0:
                results_text += "\nAdd {} weight units to {} compartment".format(
                    el, locations[idx]
                )
            else:
                results_text += "\nRemove {} weight units from {} compartment".format(
                    -el, locations[idx]
                )

        results_text += "\n\nA weight unit is ~3.8g, with the S, M, and L dowel pins\naccounting for 1, 2, and 5 units respecitvely"

        if rerun:
            results_text += "\n\n\nWeight significantly off balance,\nmake changes and test again"

        # Popup to display info in seawolf
        show_dismissable_popup(
            "Results", results_text, size_x=400, size_y=600, ok_button_fn=None, ok_button_text="Ok"
        )

    def find_mass_add(self, rise):
        # Find the total amount of mass that should be added or removed from the bot
        weight_add = rise / 1.05
        # Each weight unit is 3.8g
        weight_units = round(weight_add / 3.8)

        return weight_units

    def find_mass_dist(self, roll, pitch):
        # Find the distribution of mass that should be made (based on pitch and roll of the bot)
        # Roll analysis
        if roll <= 0:
            side_add = directions.STBD
            roll = -roll
        else:
            side_add = directions.PORT

        side_weight = math.exp((roll + 85) / 35) - 11
        side_units = round(side_weight / 3.8)

        # Pitch analysis
        if pitch <= 0:
            end_add = directions.BOW
            pitch = -pitch
        else:
            end_add = directions.STERN

        end_weight = math.exp((pitch + 35) / 19) - 6
        end_units = round(end_weight / 3.8)

        return (side_units, side_add, end_units, end_add)

    def end_buoyancy_test(self):
        """
        Displays results of test and reset buoyancy test button
        """
        rospy.loginfo("Completed buoyancy test")
        self.display_results()
        self.running = False
        self.run_buoyancy_test_button.label = "Run Buoyancy Test"

    @threaded
    def run_buoyancy_test(self, *args):
        """
        Opens popup to start buoyancy test. The test function callback is bound to the "Start" button.
        """
        # Check if running
        if self.running:
            rospy.loginfo("STOP BUOYANCY TEST")
            self.running = False
            self.run_buoyancy_test_button.label = "Run Buoyancy Test"
        else:
            popup_text = "You are about to start a buoyancy test."
            popup_text += (
                "\nDuring this test the thruster cores will all be disarmed and the bot set free."
            )
            popup_text += (
                "\nEnsure they are free of any obstructions before proceeding with the test."
            )
            popup_text += "\n\n\nNOTE: If you have just added/removed drill from cullbot,\nplease reflect with approximate opposing weights"
            self.popup = show_dismissable_popup(
                title="STARTING BUOYANCY TEST",
                text=popup_text,
                size_x=600,
                size_y=400,
                ok_button_fn=self.start_test,
                ok_button_text="Start",
            )


if __name__ == "__main__":
    rospy.init_node("buoyancy_tester", anonymous=True)

    class WidgetApp(App):
        def build(self):
            return BuoyancyTestPane()

    WidgetApp().run()
