#!/usr/bin/env python

import rospy
import threading
from std_msgs.msg import String
from debra.msg import command_msg, satellite_pose, payload_data, WOD, WOD_data

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
        self.pub_move_sat = rospy.Publisher('/move_sat', String, queue_size=10)

        # Publishers for downlink data
        self.pub_sat_pose = rospy.Publisher('/satellite_pose_data', satellite_pose, queue_size=10)
        self.pub_payload = rospy.Publisher('/payload_data', payload_data, queue_size=10)
        self.pub_downlink = rospy.Publisher('/downlink_data', String, queue_size=10)
        self.pub_wod = rospy.Publisher('/wod_data', WOD, queue_size=10)

        # Subscribers
        rospy.Subscriber('/uplink_commands', command_msg, self.uplink_callback)
        rospy.Subscriber('/debris_packet', payload_data, self.callback_debris_packet)
        rospy.Subscriber('/raw_lidar', String, self.callback_raw_lidar)
        rospy.Subscriber('/sat_info', satellite_pose, self.callback_sat_info)
        rospy.Subscriber('/current_voltage', String, self.callback_curr_volt)
        rospy.Subscriber('/temperature_data', String, self.callback_temperature)

        # Data to be sent
        # WOD
        self.wod_data = WOD()
        self.wod_data.datasets = [WOD_data() for _ in range(32)]
        self.current_wod_data = WOD_data()

        # Timer
        rospy.Timer(rospy.Duration(10), self.publish_wod)

    def uplink_callback(self, msg):
        # Check if the message matches the criteria to change state
        self.current_wod_data.satellite_mode = 1
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
            temperatures = data.data.split(',')
            time_stamp = temperatures[0]
            thermistor_temps = [float(temp) for temp in temperatures[1:5]]  # Convert the first four temperature values
            pi_cpu_temp = float(temperatures[5])  # Convert the Pi CPU temperature

            # Check temperatures for going out of the nominal range
            if any(t < -20 or t > 50 for t in thermistor_temps) or pi_cpu_temp > 85:  # Assuming 70°C as the critical temperature for Pi CPU
                if self.state != 6:
                    self.state = 6  # Safe state
                    rospy.loginfo("Temperature out of range. Switched to SAFE state.")
            elif self.state == 6 and self.nominal_battery_state:
                # Check if all temperatures are back to nominal range
                if all(-20 <= t <= 50 for t in thermistor_temps) and pi_cpu_temp <= 85:
                    self.state = 4  # Nominal state
                    rospy.loginfo("Temperature nominal. Switched to NOMINAL state.")
            self.pub_state.publish(self.STATES[self.state])
        except ValueError as e:
            rospy.logwarn(f"Invalid temperature data received: {data.data} Error: {str(e)}")

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

    def publish_wod(self, event):
        """Publish WOD every 10 seconds"""
        # Insert the current WOD_data at index 0 and shift the list
        self.wod_data.datasets.insert(0, self.current_wod_data)

        # Ensure only 32 datasets are kept
        if len(self.wod_data.datasets) > 32:
            self.wod_data.datasets.pop()

        # Reset current WOD_data for the next period
        self.current_wod_data = WOD_data()

        # Publish the WOD data if not in SAFE state
        if self.state != 6:
            self.wod_data.satellite_id = "satellite_id_placeholder"
            self.wod_data.packet_time_size = int(rospy.Time.now().to_sec())
            self.pub_wod.publish(self.wod_data)
            rospy.loginfo("Published WOD data.")

    def update_wod_data(self, field, value):
        # Insert a new dataset at index 0
        new_dataset = WOD_data()
        setattr(new_dataset, field, value)
        self.wod_data.datasets.insert(0, new_dataset)

        # Ensure only 32 datasets are kept
        if len(self.wod_data.datasets) > 32:
            self.wod_data.datasets.pop()

    def handle_user_input(self):
        while not rospy.is_shutdown():
            user_input = input("Enter new state name or number: ")  # Use input() for Python 3
            if user_input.isdigit():
                state_number = int(user_input)
                if state_number in self.STATES:
                    self.state = state_number
                    rospy.loginfo(f"State changed to: {self.STATES[self.state]}")
                else:
                    rospy.logwarn("Invalid state number.")
            else:
                state_number = next((num for num, name in self.STATES.items() if name.lower() == user_input.lower()), None)
                if state_number:
                    self.state = state_number
                    rospy.loginfo(f"State changed to: {self.STATES[self.state]}")
                else:
                    rospy.logwarn("Invalid state name.")
            self.publish_state()

    def run(self):
        # Threading for user input
        input_thread = threading.Thread(target=self.handle_user_input)
        input_thread.start()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_state()

            rate.sleep()

        input_thread.join()

if __name__ == '__main__':
    rospy.init_node('debra_node')
    debra = Debra()
    debra.run()
