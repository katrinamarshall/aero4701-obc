#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from telemetry.msg import command_msg
from transceivers import Transceiver
from transceivers import AX25UIFrame

# Callback to receive downlink data instructions
def downlink_data_callback(data: String, transceiver: Transceiver):
    info = data.data  # Information field to be filled by the user
    ssid_type = 0b1111  # Science Data

    ax25_frame = AX25UIFrame(info, ssid_type)
    frame = ax25_frame.create_frame()

    transceiver.send_deal(frame)

    # transceiver.send_deal(data.data)
    

# Function to receive data and publish to the topic
def main():
    # Initialize the ROS node
    rospy.init_node('telemetry_node')
    rospy.loginfo("Starting telemetry node")
    rate = rospy.Rate(1)

    # Initialize the transceiver object
    transceiver = Transceiver(serial_num="/dev/ttyS0", freq=433, addr=0, power=22, rssi=False, air_speed=2400, relay=False)

    # Publisher for uplink commands
    uplink_publisher = rospy.Publisher('/uplink_commands', command_msg, queue_size=10)

    # Subscriber for downlink commands
    downlink_subscriber = rospy.Subscriber('/downlink_data', String, lambda data: downlink_data_callback(data, transceiver))

    # Spin node
    while not rospy.is_shutdown():
        command = transceiver.receive_data()
        if command:
            rospy.loginfo(f"Received and publishing: component={command.component}, component_id={command.component_id}, command={command.command}")
            uplink_publisher.publish(command)
        rate.sleep()

if __name__ == '__main__':
    main()
