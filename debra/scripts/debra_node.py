#!/usr/bin/env python

import rospy
import threading
from std_msgs.msg import String, Int8
from debra.msg import command_msg, satellite_pose, payload_data, WOD, WOD_data
from eps.msg import current_voltage
from payload.msg import lidar_raw_data
from thermal.msg import temperatures

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
        self.state = 1  # Initial state is LAUNCH
        self.nominal_battery_state = True
        self.nominal_temperature_state = True
        self.satellite_calibrated = False

        # Publishers for state machine
        self.pub_state = rospy.Publisher('/operation_state', String, queue_size=10)
        self.pub_move_sat = rospy.Publisher('/move_sat', String, queue_size=10)

        # Publishers for downlink data
        self.pub_sat_pose = rospy.Publisher('/satellite_pose_data', satellite_pose, queue_size=10)
        self.pub_payload = rospy.Publisher('/payload_data', payload_data, queue_size=10)
        self.pub_downlink = rospy.Publisher('/downlink_data', String, queue_size=10)
        self.pub_wod = rospy.Publisher('/wod_data', WOD, queue_size=10)

        # Subscribers 
        rospy.Subscriber('/state_ssh_command', Int8, self.override_state)
        rospy.Subscriber('/debris_packet', payload_data, self.callback_debris_packet)
        rospy.Subscriber('/sat_info', satellite_pose, self.callback_sat_info)
        rospy.Subscriber('/uplink_commands', command_msg, self.uplink_callback)
        rospy.Subscriber('/raw_lidar_data', lidar_raw_data, self.callback_raw_lidar)
        rospy.Subscriber('/current_voltage', current_voltage, self.callback_curr_volt)
        rospy.Subscriber('/temperatures', temperatures, self.callback_temperature)

        # Data to be sent
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
        
        # THIS IS AN EXAMPLE OF WOD WORKING AND MUST BE REMOVED
        if msg.component == 'o' and msg.component_id == 1:
            self.current_wod_data.temperature_comm = 15.6

        self.pub_state.publish(self.STATES[self.state])

    def override_state(self, data):
        rospy.loginfo("Overriding state...")
        new_state = data.data
        try:
            state_number = int(new_state)
            if state_number in self.STATES:
                self.state = state_number
                rospy.loginfo(f"State changed to: {self.STATES[self.state]}")
                self.pub_state.publish(self.STATES[self.state])
        except ValueError:
            rospy.logwarn(f"Invalid state number received: {new_state}")

##----------------- TO DO ------------------
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

    def callback_raw_lidar(self, data):
        # Process raw lidar data (this is just a placeholder)
        rospy.loginfo(f"Received raw lidar data!")
#--------------------------------------------

    def callback_temperature(self, data):
        try:
            pi_cpu_temp = data.pi
            thermistor_temps = [data.thermistor_1, data.thermistor_2]
            
            # Check if temp exceeds the nominal range
            if any(t < -20 or t > 50 for t in thermistor_temps) or pi_cpu_temp > 85:  # Assuming 85°C as the critical temperature for Pi CPU
                self.nominal_temperature_state = False
                rospy.loginfo("WARNING Temperature out of range.")
            else: 
                self.nominal_temperature_state = True
            self.check_battery_and_temp()

            # FOR MING TO CHANGE LATER
            self.current_wod_data.temperature_comm = thermistor_temps[0]
            self.current_wod_data.temperature_eps = thermistor_temps[1]
            self.current_wod_data.temperature_battery = thermistor_temps[1] # does not exisst

        except ValueError as e:
            rospy.logwarn(f"Invalid temperature data received: {data.data} Error: {str(e)}")


    def callback_curr_volt(self, data):
        try:
            ############# TO CHANGE TO ACTUAL BATTERY VOLTAGE ############
            if data.voltage_40 < 0.25:
                self.nominal_battery_state = False
                rospy.loginfo("WARNING Battery voltage low.")
            elif self.state == 6:
                self.nominal_battery_state = True
            self.check_battery_and_temp()

            # FOR LUCAS TO CHANGE LATER
            self.current_wod_data.battery_voltage = data.voltage_40
            self.current_wod_data.battery_current = data.current_40
            self.current_wod_data.regulated_bus_current_3v3 = data.current_44
            self.current_wod_data.regulated_bus_current_5v = data.current_41

        except ValueError:
            rospy.logwarn(f"Invalid battery voltage data received: {data.data}")

    def check_battery_and_temp(self):
        if self.state == 6:
            if self.nominal_battery_state and self.nominal_temperature_state:
                self.state = 4
                rospy.loginfo("Back to nominal operation. Switched to NOMINAL state.")
                self.pub_state.publish(self.STATES[self.state])
        else:
            if not self.nominal_battery_state or not self.nominal_temperature_state:
                self.state = 6
                rospy.loginfo("Back to nominal operation. Switched to SAFE state.")
                self.pub_state.publish(self.STATES[self.state])

    def publish_wod(self, event):
        """Publish WOD every 10 seconds"""
        # Update satellite mode field
        print(f"State: {self.state}")
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
            self.pub_wod.publish(self.wod_data)
            rospy.loginfo("Published WOD data.")

            # Publish satellite pose
            self.pub_sat_pose.publish(self.sat_pose)


if __name__ == '__main__':
    rospy.init_node('debra_node')
    rospy.loginfo("Launching DEBRA")
    debra = Debra()
    rospy.spin()