#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from telemetry.msg import command_msg
from transceivers import Transceiver
from AX25UI import AX25UIFrame
from debra.msg import payload_data, satellite_pose, WOD_data, WOD 
import struct

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
        # Pack the time field in little endian
        time_field = struct.pack('<I', data.packet_time_size)

        # Pack each data set
        if len(data.datasets) <= 32:
            datasets = b''.join(pack_wod_dataset(dataset) for dataset in data.datasets)
        
        # For when there is more than 32 datasets, just take the first 32
        else:
            datasets_to_pack = data.datasets[:32]
            datasets = b''.join(pack_wod_dataset(dataset) for dataset in datasets_to_pack)

        print(f"Number of datasets: {len(datasets)}")
        # # Pad if missing datasets
        # if len(data.datasets) <= 32:
        #     packets_padding = len(data.datasets) % 8 
        expected_length = 32 * 57 // 8
        if len(datasets) < expected_length:
            datasets = datasets.ljust(expected_length, b'\x00') # Pads with zeroes

        # Combine time field and datasets
        info = time_field + datasets

        print(f"Packed WOD Data: {info.hex()} (Length: {len(info)} bytes)")
        ssid_type = 0b1110  # WOD data type

        # Create an ax.25 UI frame
        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        # Send data
        self.transceiver.send_deal(frame)


    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            command = self.transceiver.receive_data()
            if command:
                rospy.loginfo(f"Received and publishing: component={command.component}, component_id={command.component_id}, command={command.command}")
                self.uplink_publisher.publish(command)
            rate.sleep()



# Helper functions ----------------------------------------------------------------
def pack_wod_dataset(dataset):
    """Pack a single WOD data set into binary format."""
    return struct.pack('B'*8, 
        dataset.satellite_mode,
        dataset.battery_voltage,
        dataset.battery_current,
        dataset.regulated_bus_current_3v3,
        dataset.regulated_bus_current_5v,
        dataset.temperature_comm,
        dataset.temperature_eps,
        dataset.temperature_battery
    )

# def pack_wod_data(dataset):
#     # Combine the fields into a single 57-bit integer
#     combined = (dataset.satellite_mode << 56) | (dataset.battery_voltage << 48) | (dataset.battery_current << 40) | (dataset.regulated_bus_current_3v3 << 32) | (dataset.regulated_bus_current_5v << 24) | (dataset.temperature_comm << 16) | (dataset.temperature_eps << 8) | dataset.temperature_battery
    
#     # Convert the 57-bit integer into a bytearray
#     packed_data = bytearray()
#     for i in range(7, -1, -1):  # 7 bytes for 56 bits, but we only need 7 bytes for 57 bits
#         packed_data.append((combined >> (i * 8)) & 0xFF)
    
#     return packed_data


if __name__ == '__main__':
    telemetry = Telemetry()
    telemetry.run()
