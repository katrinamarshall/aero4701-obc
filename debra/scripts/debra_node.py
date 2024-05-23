#!/usr/bin/env python

import rospy
import threading
from std_msgs.msg import String
from debra.msg import command_msg, satellite_pose, payload_data, WOD, WOD_data
import os
import json

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

        # Telemetry data
        self.store_data = True
        self.wod_file_path = os.path.join(os.path.expanduser('~'), 'catkin_ws/src/aero4701-obc/debra/Data/wod_data.json')

        # WOD
        self.wod_data = WOD()
        self.wod_data.datasets = [WOD_data() for _ in range(32)]
        self.current_wod_data = WOD_data()

        # Satellite pose
        self.sat_pose = satellite_pose()

        # Payload data
        self.payload = payload_data()

        # Timer
        rospy.Timer(rospy.Duration(10), self.publish_wod)


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
        
        # Ready to send data WOD to telemetry
        elif msg.component == 'c' and msg.component_id == '1':
            self.store_data = False
        
        # Connection to telemetry lost, have to store WOD data on board
        elif msg.component == 'c' and msg.component_id == '0':
            self.store_data = True
        
        # THIS IS AN EXAMPLE OF WOD WORKING AND MUST BE REMOVED
        if msg.component == 'o' and msg.component_id == 1:
            self.current_wod_data.temperature_comm = 15.6

        self.publish_state()


    def callback_debris_packet(self, data):
        # Define the attributes for payload_data
        payload_attributes = [
            'debris_position_x', 'debris_position_y', 'debris_position_z',
            'debris_velocity_x', 'debris_velocity_y', 'debris_velocity_z',
            'debris_diameter', 'time_of_detection', 'object_count'
        ]

        # Update payload data
        for attr in payload_attributes:
            setattr(self.payload, attr, getattr(data, attr))

        # Publish the updated payload data
        self.pub_payload.publish(self.payload)
        rospy.loginfo("Published updated payload data.")


    def callback_raw_lidar(self, data):
        # Process raw lidar data (this is just a placeholder)
        rospy.loginfo(f"Received raw lidar data: {data}")


    def callback_sat_info(self, data):
        # # Check if satellite information matches the desired state
        # if not self.satellite_calibrated:
        #     self.state = 3  # Calibration state
        #     self.pub_state.publish(self.STATES[self.state])
        # else:
        #     # Find required controls to orient the satellite
        #     self.pub_move_sat.publish("Orientation Command")
        #     rospy.loginfo("Published move_sat command.")


        # # Update satellite pose data
        attributes = [
            'position_x', 'position_y', 'position_z',
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            'velocity_x', 'velocity_y', 'velocity_z'
        ]
        for attr in attributes:
            setattr(self.sat_pose, attr, getattr(data, attr))


    def callback_temperature(self, data):
        try:
            temperatures = data.data.split(',')
            # time_stamp = temperatures[0]
            thermistor_temps = [float(temp) for temp in temperatures[0:3]]  # Convert the three thermistor temperature values
            pi_cpu_temp = float(temperatures[3])  

            # Check if temp exceeds the nominal range
            if any(t < -20 or t > 50 for t in thermistor_temps) or pi_cpu_temp > 85:  # Assuming 85°C as the critical temperature for Pi CPU
                if self.state != 6:
                    self.state = 6  # Safe state
                    rospy.loginfo("Temperature out of range. Switched to SAFE state.")
            elif self.state == 6 and self.nominal_battery_state:
                # Check if all temperatures are back to nominal range
                if all(-20 <= t <= 50 for t in thermistor_temps) and pi_cpu_temp <= 85:
                    self.state = 4  # Nominal state
                    rospy.loginfo("Temperature nominal. Switched to NOMINAL state.")
            self.pub_state.publish(self.STATES[self.state])

            # FOR MING TO CHANGE LATER
            self.current_wod_data.temperature_comm = thermistor_temps[0]
            self.current_wod_data.temperature_eps = thermistor_temps[1]
            self.current_wod_data.temperature_battery = thermistor_temps[2]

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

            # FOR LUCAS TO CHANGE LATER
            self.current_wod_data.battery_voltage = 3.5
            self.current_wod_data.battery_current = 0.8
            self.current_wod_data.regulated_bus_current_3v3 = 0.5
            self.current_wod_data.regulated_bus_current_5v = 0.7

        except ValueError:
            rospy.logwarn(f"Invalid battery voltage data received: {data.data}")


    def publish_state(self):
        # Publish the current state to /operation_state
        # rospy.loginfo(f"Current State: {self.STATES[self.state]}")
        self.pub_state.publish(self.STATES[self.state])


    def publish_wod(self, event):
        """Publish WOD every 10 seconds"""
        # Update satellite mode field
        if self.state == 5:
            self.current_wod_data.satellite_mode = 1
        else:
            self.current_wod_data.satellite_mode = 0

        # Insert the current WOD_data at index 0 and shift the list
        self.wod_data.datasets.insert(0, self.current_wod_data)

        # Ensure only 32 datasets are kept
        if len(self.wod_data.datasets) > 32:
            self.wod_data.datasets.pop()

        # Reset current WOD_data for the next period
        self.current_wod_data = WOD_data()

        # Publish the WOD data if not in SAFE state
        if self.state != 6:
            # Setting ID
            self.wod_data.satellite_id = "DEBRA"

            # Reference epoch (01/01/2000 00:00:00 UTC)
            reference_epoch = rospy.Time(946684800)  # 946684800 seconds since 1970-01-01 00:00:00 UTC (UNIX time) to 2000-01-01 00:00:00 UTC

            # Calculate seconds since the reference epoch (01/01/2000 00:00:00 UTC)
            current_time = rospy.Time.now()
            elapsed_seconds = (current_time - reference_epoch).to_sec()

            # Set the wod time
            self.wod_data.packet_time_size = int(elapsed_seconds)

            # Publish wod
            if self.store_data == False:
                self.pub_wod.publish(self.wod_data)
                rospy.loginfo("Published WOD data.")
            
            # Store wod data
            elif self.store_data == True:
                rospy.loginfo("Storing WOD data")
                self.append_wod_data_to_file()
                rospy.loginfo("Stored WOD data")

            # Publish satellite pose
            self.pub_sat_pose.publish(self.sat_pose)


    def append_wod_data_to_file(self):
        """Appends wod data to a json when groundstation is not connected"""
        if self.wod_data:
            # Create the data dictionary to be appended
            wod_entry = {
                "satellite_id": self.wod_data.satellite_id,
                "packet_time_size": self.wod_data.packet_time_size,
                "datasets": [
                    {
                        "satellite_mode": dataset.satellite_mode,
                        "battery_voltage": dataset.battery_voltage,
                        "battery_current": dataset.battery_current,
                        "regulated_bus_current_3v3": dataset.regulated_bus_current_3v3,
                        "regulated_bus_current_5v": dataset.regulated_bus_current_5v,
                        "temperature_comm": dataset.temperature_comm,
                        "temperature_eps": dataset.temperature_eps,
                        "temperature_battery": dataset.temperature_battery
                    }
                    for dataset in self.wod_data.datasets
                ]
            }

            # Check if the file exists, if not create an empty list
            if not os.path.isfile(self.wod_file_path):
                data_list = []
            else:
                try:
                    with open(self.file_path, 'r') as file:
                        data_list = json.load(file)
                # If the file is empty
                except json.JSONDecodeError:
                    data_list = []

            # Append the new entry
            data_list.append(wod_entry)

            # Write back to the file
            with open(self.wod_file_path, 'w') as file:
                json.dump(data_list, file, indent=4)
            rospy.loginfo("WOD data appended to file")



    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_state()

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('debra_node')
    rospy.loginfo("Launching DEBRA")
    debra = Debra()
    debra.run()
