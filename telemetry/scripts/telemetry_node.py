#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from telemetry.msg import command_msg
from transceivers import Transceiver
from AX25UI import AX25UIFrame
from debra.msg import payload_data, satellite_pose, WOD_data, WOD
import struct
import math
import queue
import threading

class Telemetry:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('telemetry_node')
        rospy.loginfo("Starting telemetry node")

        # Initialize the transceiver object
        self.transceiver = Transceiver(serial_num="/dev/ttyS0", freq=433, addr=0, power=22, rssi=False, air_speed=2400, relay=False)

        # Publisher for uplink commands
        self.uplink_publisher = rospy.Publisher('/uplink_commands', command_msg, queue_size=10)

        # Subscribers for different data
        rospy.Subscriber('/downlink_data', String, self.downlink_data_callback)
        rospy.Subscriber('/payload_data', payload_data, self.payload_data_callback)
        rospy.Subscriber('/satellite_pose_data', satellite_pose, self.satellite_pose_data_callback)
        rospy.Subscriber('/wod_data', WOD, self.wod_data_callback)

        # Initialize a queue for the messages
        self.message_queue = queue.Queue()

        # Initialize a lock for sending messages
        self.send_lock = threading.Lock()

        # Start a thread to process the queue
        self.processing_thread = threading.Thread(target=self.process_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def downlink_data_callback(self, data):
        """Enqueues a miscellaneous message to be sent by the transceiver"""
        self.message_queue.put(('downlink', data))

    def payload_data_callback(self, data):
        """Enqueues payload (science) data to be sent by the transceiver"""
        self.message_queue.put(('payload', data))

    def satellite_pose_data_callback(self, data):
        """Enqueues satellite pose data to be sent by the transceiver"""
        self.message_queue.put(('satellite_pose', data))

    def wod_data_callback(self, data):
        """Enqueues WOD data to be sent by the transceiver"""
        self.message_queue.put(('wod', data))

    def process_queue(self):
        while not rospy.is_shutdown():
            if not self.message_queue.empty():
                message_type, data = self.message_queue.get()
                with self.send_lock:
                    if message_type == 'downlink':
                        self.process_downlink_data(data)
                    elif message_type == 'payload':
                        self.process_payload_data(data)
                    elif message_type == 'satellite_pose':
                        self.process_satellite_pose_data(data)
                    elif message_type == 'wod':
                        self.process_wod_data(data)

    def process_downlink_data(self, data):
        info = data.data
        ssid_type = 0b1011  # Misc ssid

        ax25_frame = AX25UIFrame(info.encode('ascii'), ssid_type)
        frame = ax25_frame.create_frame()

        self.transceiver.send_deal(frame)

    def process_payload_data(self, data):
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
        ssid_type = 0b1111  # Science ssid

        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        self.transceiver.send_deal(frame)

    def process_satellite_pose_data(self, data):
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
        ssid_type = 0b1101  # Satellite_pose ssid

        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        self.transceiver.send_deal(frame)

    def process_wod_data(self, data):
        id_field = struct.pack('5s', data.satellite_id.encode('ascii'))
        time_field = struct.pack('<I', data.packet_time_size)

        first_16_datasets = data.datasets[:16]
        second_16_datasets = data.datasets[16:32]
        first_wod = struct.pack('B', 1)
        second_wod = struct.pack('B', 2)

        first_16_datasets_packed = b''.join(pack_wod_dataset(dataset) for dataset in first_16_datasets)
        expected_length = 16 * 64 // 8
        if len(first_16_datasets_packed) < expected_length:
            first_16_datasets_packed = first_16_datasets_packed.ljust(expected_length, b'\x00')

        first_frame_info = first_wod + id_field + time_field + first_16_datasets_packed

        ssid_type = 0b1110  # WOD data type
        first_ax25_frame = AX25UIFrame(first_frame_info, ssid_type)
        first_frame = first_ax25_frame.create_frame()
        self.transceiver.send_deal(first_frame)

        second_16_datasets_packed = b''.join(pack_wod_dataset(dataset) for dataset in second_16_datasets)
        expected_length = 16 * 64 // 8
        if len(second_16_datasets_packed) < expected_length:
            second_16_datasets_packed = second_16_datasets_packed.ljust(expected_length, b'\x00')

        second_frame_info = second_wod + second_16_datasets_packed

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
def convert_voltage(voltage):
    return max(0, min(255, math.floor((20 * voltage) - 60)))

def convert_current(current):
    return max(0, min(255, math.floor(127 * current) + 127))

def convert_bus_current(current):
    return max(0, min(255, math.floor(40 * current)))

def convert_temperature(temp):
    return max(0, min(255, math.floor((4 * temp) + 60)))

def pack_wod_dataset(dataset):
    satellite_mode = 1 if dataset.satellite_mode else 0
    battery_voltage = convert_voltage(dataset.battery_voltage)
    battery_current = convert_current(dataset.battery_current)
    regulated_bus_current_3v3 = convert_bus_current(dataset.regulated_bus_current_3v3)
    regulated_bus_current_5v = convert_bus_current(dataset.regulated_bus_current_5v)
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
