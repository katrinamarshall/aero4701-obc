#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from telemetry.msg import command_msg
from transceivers import Transceiver
from AX25UI import AX25UIFrame
from debra.msg import payload_data, satellite_pose, WOD_data, WOD 
import struct
import math

class Telemetry:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('telemetry_node')
        rospy.loginfo("Starting telemetry node")
        
        # Initialize the transceiver object
        self.transceiver = Transceiver(serial_num="/dev/ttyS0", freq=433, addr=0, power=22, rssi=False, air_speed=2400, relay=False)
        
        # Publisher for uplink commands
        self.uplink_publisher = rospy.Publisher('/uplink_commands', command_msg, queue_size=10)
        
        # Subscriber for downlink commands
        rospy.Subscriber('/downlink_data', String, self.downlink_data_callback)
        
        # Subscriber for payload data
        rospy.Subscriber('/payload_data', payload_data, self.payload_data_callback)

        # Subscriber for statellite pose data
        rospy.Subscriber('/satellite_pose_data', satellite_pose, self.satellite_pose_data_callback)

        # Subscriber for WOD data
        rospy.Subscriber('/wod_data', WOD, self.wod_data_callback)

    def downlink_data_callback(self, data):
        """Sends a miscellaneous message to be sent by the transceiver"""
        info = data.data  
        ssid_type = 0b1110

        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        self.transceiver.send_deal(frame)

    def payload_data_callback(self, data):
        """Packs and sends payload (science) data"""
        info = struct.pack('<fff fff f i i',
            data.debris_position_x,
            data.debris_position_y,
            data.debris_position_z,
            data.debris_velocity_x,
            data.debris_velocity_y,
            data.debris_velocity_z,
            data.debris_diameter,
            data.time_of_detection,
            data.object_count
        )
        print(f"Science data: {info.hex()}")
        ssid_type = 0b1111

        # Create an ax.25 UI frame
        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        # Send data
        self.transceiver.send_deal(frame)


    def satellite_pose_data_callback(self, data):
        """Packs and sends payload (science) data"""
        info = struct.pack('<fff ffff fff',
            data.position_x,
            data.position_y,
            data.position_z,
            data.orientation_x,
            data.orientation_y,
            data.orientation_z,
            data.orientation_w,
            data.velocity_x,
            data.velocity_y,
            data.velocity_z
        )
        print(f"Satellite pose data: {info.hex()}")
        ssid_type = 0b1101

        # Create an ax.25 UI frame
        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        # Send data
        self.transceiver.send_deal(frame)



    def wod_data_callback(self, data):
        """Packs and sends WOD data"""
        # Pack "DEBRA" satellite id
        id_field = struct.pack('5s', data.satellite_id.encode('ascii'))

        # Pack the time field in little endian
        time_field = struct.pack('<I', data.packet_time_size)

        # Split datasets into two parts
        first_16_datasets = data.datasets[:16]
        second_16_datasets = data.datasets[16:32]
        first_wod = struct.pack('B', 1)
        second_wod = struct.pack('B', 2)

        # Pack the first 16 datasets
        first_16_datasets_packed = b''.join(pack_wod_dataset(dataset) for dataset in first_16_datasets)
        expected_length = 16 * 64 // 8  
        if len(first_16_datasets_packed) < expected_length:
            first_16_datasets_packed = first_16_datasets_packed.ljust(expected_length, b'\x00')  # Pad with zeroes

        # Combine id_field, time_field, and the first 16 datasets
        first_frame_info = first_wod + id_field + time_field + first_16_datasets_packed
        print(f"Packed WOD Data (first frame): {first_frame_info.hex()} (Length: {len(first_frame_info)} bytes)")

        # Create and send the first ax.25 UI frame
        ssid_type = 0b1110  # WOD data type
        first_ax25_frame = AX25UIFrame(first_frame_info, ssid_type)
        first_frame = first_ax25_frame.create_frame()
        self.transceiver.send_deal(first_frame)

        # Pack the second 16 datasets
        second_16_datasets_packed = b''.join(pack_wod_dataset(dataset) for dataset in second_16_datasets)
        expected_length = 16 * 64 // 8  # 16 datasets, each 57 bits, converted to bytes
        if len(second_16_datasets_packed) < expected_length:
            second_16_datasets_packed = second_16_datasets_packed.ljust(expected_length, b'\x00')  # Pad with zeroes if needed

        # Combine only the second 16 datasets
        second_frame_info = second_wod + second_16_datasets_packed
        print(f"Packed WOD Data (second frame): {second_frame_info.hex()} (Length: {len(second_frame_info)} bytes)")

        # Create and send the second ax.25 UI frame
        second_ax25_frame = AX25UIFrame(second_frame_info, ssid_type)
        second_frame = second_ax25_frame.create_frame()
        self.transceiver.send_deal(second_frame)



    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            command = self.transceiver.receive_data()
            if command:
                rospy.loginfo(f"Received and publishing: component={command.component}, component_id={command.component_id}, command={command.command}")
                self.uplink_publisher.publish(command)
            rate.sleep()



# Helper functions ----------------------------------------------------------------
# def pack_wod_dataset(dataset):
#     """Pack a single WOD data set into binary format."""
#     return struct.pack('B'*8, 
#         dataset.satellite_mode,
#         dataset.battery_voltage,
#         dataset.battery_current,
#         dataset.regulated_bus_current_3v3,
#         dataset.regulated_bus_current_5v,
#         dataset.temperature_comm,
#         dataset.temperature_eps,
#         dataset.temperature_battery
#     )

def convert_voltage(voltage):
    return max(0, min(255, math.floor((20 * voltage) - 60)))

def convert_current(current):
    return max(0, min(255, math.floor(127 * current) + 127))

def convert_bus_current(current):
    return max(0, min(255, math.floor(40 * current)))

def convert_temperature(temp):
    return max(0, min(255, math.floor((4 * temp) + 60)))

def pack_wod_dataset(dataset):
    """Pack a single WOD data set into binary format."""
    # Convert all floats to unsigned 8bit integers as per WOD format
    # Mode
    satellite_mode = 1 if dataset.satellite_mode else 0

    # Battery voltage
    battery_voltage = convert_voltage(dataset.battery_voltage)

    # Battery current
    battery_current = convert_current(dataset.battery_current)

    # Bus currents
    regulated_bus_current_3v3 = convert_bus_current(dataset.regulated_bus_current_3v3)
    regulated_bus_current_5v = convert_bus_current(dataset.regulated_bus_current_5v)

    # Temperatures
    temperature_comm = convert_temperature(dataset.temperature_comm)
    temperature_eps = convert_temperature(dataset.temperature_eps)
    temperature_battery = convert_temperature(dataset.temperature_battery)

    return struct.pack('B'*8,
        satellite_mode,
        battery_voltage,
        battery_current,
        regulated_bus_current_3v3,
        regulated_bus_current_5v,
        temperature_comm,
        temperature_eps,
        temperature_battery
    )



if __name__ == '__main__':
    telemetry = Telemetry()
    telemetry.run()
