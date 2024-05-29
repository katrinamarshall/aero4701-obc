#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from telemetry.msg import command_msg
from transceivers import Transceiver
from AX25UI import AX25UIFrame
from debra.msg import payload_data, satellite_pose, WOD_data, WOD
from payload.msg import lidar_raw_data, found_debris
import struct
import math
import queue

class Telemetry:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('telemetry_node')
        
        # Initialize the Transceiver object
        self.transceiver = Transceiver(serial_num="/dev/ttyS0", freq=433, addr=0, power=22, rssi=False, air_speed=2400, relay=False)
        # Publisher for uplink commands
        self.uplink_publisher = rospy.Publisher('/uplink_commands', command_msg, queue_size=10)
        
        # Subscribers for downlinking different message types
        rospy.Subscriber('/downlink_data', String, self.downlink_data_callback)
        rospy.Subscriber('/payload_data', payload_data, self.payload_data_callback)
        rospy.Subscriber('/satellite_pose_data', satellite_pose, self.satellite_pose_data_callback)
        rospy.Subscriber('/wod_data', WOD, self.wod_data_callback)
        rospy.Subscriber('/raw_lidar_data', lidar_raw_data, self.raw_lidar_callback)
        rospy.Subscriber('/found_debris', found_debris, self.found_debris_callback)

        # Message queue for processing outgoing messages
        self.message_queue = queue.Queue()

        # Timer for periodic processing
        rospy.Timer(rospy.Duration(1.7), self.timer_callback)

        # Counter for raw lidar messages
        self.lidar_counter = 0

        # When to send the raw data
        self.store_raw = False

    def timer_callback(self, event):
        """Periodic timer callback for processing messages"""
        if not self.message_queue.empty():
            frame = self.message_queue.get()

            # Send the frame to the transceiver to be transmitted
            self.transceiver.send_deal(frame)

    def downlink_data_callback(self, data):
        """Callback for downlink data"""
        info = data.data  
        ssid_type = 0b1011 # Misc ssid

        # Misc data is a string so must be ascii encoded
        ax25_frame = AX25UIFrame(info.encode('ascii'), ssid_type)
        frame = ax25_frame.create_frame()

        # Put message frame into queue
        self.message_queue.put(frame)

    def payload_data_callback(self, data):
        """Callback for payload (science) data"""
        # Pack data according to payload_data msg type
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
        ssid_type = 0b1111 # Science ssid

        # Create ax.25 frame
        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        # Put message frame into queue 
        self.message_queue.put(frame)

    def satellite_pose_data_callback(self, data):
        """Callback for satellite pose data"""
        # Pack data according to satellite_pose msg type
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
        ssid_type = 0b1101 # Satellite_pose ssid

        # Create ax.25 frame
        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        # Put mesage frame into queue
        self.message_queue.put(frame)

    def raw_lidar_callback(self, data):
        """Callback for raw lidar data"""
        # # Increment the counter
        # self.lidar_counter += 1

        # # Only process every 10th message
        # if self.lidar_counter % 10 == 0:

        if self.store_raw:
            # Iterate over the four distance arrays
            for distance_num in range(1, 5):
                # Get the distance array from lidar number
                distance_array = getattr(data, f'distances_{distance_num}')

                if len(distance_array) != 0:
                    # Create data packet
                    lidar_num = struct.pack('B', distance_num)
                    lidar_data = struct.pack('64H', *distance_array)
                    # lidar_data = struct.pack(f'{len(distance_array)}H', *distance_array)

                    empty = struct.pack('B', 0)
                    # Create frame
                    frame_info = lidar_num + lidar_data + empty
                    # print(f"Frame size: {struct.calcsize(frame_info)}")
                    #print(f"Frame size: {len(frame_info)} bytes")
                    ssid_type = 0b1100  # Raw lidar data type
                    frame = AX25UIFrame(frame_info, ssid_type)

                    # Put message into frame
                    # print(f"Putting the raw_lidar frame into the queue: {frame_info}")
                    self.message_queue.put(frame.create_frame())

    def found_debris_callback(self, data):
        """Callback for debris messages"""
        # Process data according to found_debris msg type
        info = struct.pack('<B f f f f B', 
            data.lidar_label,
            data.range,
            data.theta,
            data.phi,
            data.size,
            data.num_pixels
        ) + bytearray(data.blob_position)
        
        ssid_type = 0b1000  # Debris ssid

        # Create ax.25 frame
        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        # Put message frame into queue
        self.message_queue.put(frame)
        self.store_raw = True
        rospy.sleep(0.4)
        self.store_raw = False


    def wod_data_callback(self, data):
        """Callback for WOD data"""
        # Pack data according to wod msg type
        id_field = struct.pack('5s', data.satellite_id.encode('ascii'))
        time_field = struct.pack('<I', data.packet_time_size)

        # WOD is big, so sends in two separate frames
        first_16_datasets = data.datasets[:16]
        second_16_datasets = data.datasets[16:32]
        first_wod = struct.pack('B', 1)
        second_wod = struct.pack('B', 2)
        first_16_datasets_packed = b''.join(pack_wod_dataset(dataset) for dataset in first_16_datasets)

        # Pad with 0's if missing info
        expected_length = 16 * 64 // 8  
        if len(first_16_datasets_packed) < expected_length:
            first_16_datasets_packed = first_16_datasets_packed.ljust(expected_length, b'\x00')

        # Create ax.25 for first frame
        first_frame_info = first_wod + id_field + time_field + first_16_datasets_packed
        ssid_type = 0b1110  # WOD data type
        first_ax25_frame = AX25UIFrame(first_frame_info, ssid_type)
        first_frame = first_ax25_frame.create_frame()

        # Put first frame into queue
        self.message_queue.put(first_frame)

        # Second frame
        second_16_datasets_packed = b''.join(pack_wod_dataset(dataset) for dataset in second_16_datasets)
        expected_length = 16 * 64 // 8
        if len(second_16_datasets_packed) < expected_length:
            second_16_datasets_packed = second_16_datasets_packed.ljust(expected_length, b'\x00')

        # Create ax.25 for second frame
        second_frame_info = second_wod + second_16_datasets_packed
        second_ax25_frame = AX25UIFrame(second_frame_info, ssid_type)
        second_frame = second_ax25_frame.create_frame()

        # Put second frame into queue
        self.message_queue.put(second_frame)

    def run(self):
        """Main loop for the telemetry ROS node"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Constantly listen for groundstation commands
            command = self.transceiver.receive_data()

            # Publish commands to /uplink_data
            if command:
                rospy.loginfo(f"Received and publishing: component={command.component}, component_id={command.component_id}, command={command.command}")
                self.uplink_publisher.publish(command)
            rate.sleep()

# Helper functions ----------------------------------------------------------------
def convert_voltage(voltage):
    # Converts voltage to a byte value
    return max(0, min(255, math.floor((20 * voltage) - 60)))

def convert_current(current):
    # Converts current to a byte value
    return max(0, min(255, math.floor(127 * current) + 127))

def convert_bus_current(current):
    # Converts bus current to a byte value
    return max(0, min(255, math.floor(40 * current)))

def convert_temperature(temp):
    # Converts temperature to a byte value
    return max(0, min(255, math.floor((4 * temp) + 60)))

def pack_wod_dataset(dataset):
    """Packs WOD dataset into a byte structure"""
    # Single bit for satellite_mode
    satellite_mode = 1 if dataset.satellite_mode else 0

    # Use helper functions from given WOD data format to pack floats into 
    # 8 bit unsigned integers
    # Battery
    battery_voltage = convert_voltage(dataset.battery_voltage)
    battery_current = convert_current(dataset.battery_current)

    # Regulated bus current
    regulated_bus_current_3v3 = convert_bus_current(dataset.regulated_bus_current_3v3)
    regulated_bus_current_5v = convert_bus_current(dataset.regulated_bus_current_5v)

    # Temperature
    temperature_comm = convert_temperature(dataset.temperature_comm)
    temperature_eps = convert_temperature(dataset.temperature_eps)
    temperature_battery = convert_temperature(dataset.temperature_battery)

    # Return packed WOD data as 8bytes
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
    # Create Telemetry object and run the node
    telemetry = Telemetry()
    telemetry.run()
