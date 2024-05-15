#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from debra.msg import command_msg, satellite_pose, payload_data

class Debra:
    STATES = {
        1: "Launch",
        2: "Detumble",
        3: "Calibration",
        4: "Nominal",
        5: "Debris Detection",
        6: "Safe",
        7: "Payload Pointing",
        8: "Deorbit"
    }

    def __init__(self):
        self.state = 2  # Initial state is DETUMBLE
        self.nominal_battery_state = True
        self.nominal_temperature_state = True
        self.satellite_calibrated = False

        # Publishers
        self.pub_state = rospy.Publisher('/operation_state', String, queue_size=10)
        self.pub_downlink = rospy.Publisher('/downlink_data', String, queue_size=10)
        self.pub_move_sat = rospy.Publisher('/move_sat', String, queue_size=10)

        # Additional publishers for the example
        self.pose_pub = rospy.Publisher('/satellite_pose', satellite_pose, queue_size=10)
        self.payload_pub = rospy.Publisher('/payload_data', payload_data, queue_size=10)

        # Subscribers
        rospy.Subscriber('/uplink_commands', command_msg, self.uplink_callback)
        rospy.Subscriber('/debris_packet', payload_data, self.callback_debris_packet)
        rospy.Subscriber('/raw_lidar', String, self.callback_raw_lidar)
        rospy.Subscriber('/sat_info', satellite_pose, self.callback_sat_info)
        rospy.Subscriber('/current_voltage', String, self.callback_curr_volt)
        rospy.Subscriber('/temperature', String, self.callback_temperature)

    def uplink_callback(self, msg):
        # Check if the message matches the criteria to change state
        if msg.component == 'o' and msg.component_id == 0:
            try:
                state_number = int(msg.command)
                if state_number in self.STATES:
                    self.state = state_number
                    rospy.loginfo(f"State changed to: {self.STATES[self.state]}")
            except ValueError:
                rospy.logwarn(f"Invalid state number received: {msg.command}")

        self.publish_state()

    def callback_debris_packet(self, data):
        # Process debris packet and publish to downlink
        self.pub_downlink.publish(f"Debris Packet: {data}")
        rospy.loginfo("Published debris packet to downlink.")

    def callback_raw_lidar(self, data):
        # Process raw lidar data (this is just a placeholder)
        rospy.loginfo(f"Received raw lidar data: {data}")

    def callback_sat_info(self, data):
        # Check if satellite information matches the desired state
        if not self.satellite_calibrated:
            self.state = 3  # Calibration state
            self.pub_state.publish(self.STATES[self.state])
        else:
            # Find required controls to orient the satellite
            self.pub_move_sat.publish("Orientation Command")
            rospy.loginfo("Published move_sat command.")

    def callback_temperature(self, data):
        try:
            temperature = float(data.data)
            if temperature < -20 or temperature > 50:
                self.state = 6  # Safe state
                rospy.loginfo("Temperature out of range. Switched to SAFE state.")
            elif self.state == 6 and self.nominal_battery_state:
                self.state = 4  # Nominal state
                rospy.loginfo("Temperature nominal. Switched to NOMINAL state.")
            self.pub_state.publish(self.STATES[self.state])
        except ValueError:
            rospy.logwarn(f"Invalid temperature data received: {data.data}")

    def callback_curr_volt(self, data):
        try:
            battery = float(data.data)
            if battery < 0.25:
                self.state = 6  # Safe state
                rospy.loginfo("Battery voltage low. Switched to SAFE state.")
            elif self.state == 6 and self.nominal_temperature_state:
                self.state = 4  # Nominal state
                rospy.loginfo("Battery voltage nominal. Switched to NOMINAL state.")
            self.pub_state.publish(self.STATES[self.state])
        except ValueError:
            rospy.logwarn(f"Invalid battery voltage data received: {data.data}")

    def publish_state(self):
        # Publish the current state to /operation_state
        rospy.loginfo(f"Current State: {self.STATES[self.state]}")
        self.pub_state.publish(self.STATES[self.state])

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_state()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('debra_node')
    debra = Debra()
    debra.run()
