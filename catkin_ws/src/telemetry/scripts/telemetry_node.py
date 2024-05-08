# File: telemetry/scripts/telemetry_node.py
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from telemetry.msg import command_msg
from telemetry.src.transceiver.transceiver import Transceiver

# Initialize the ROS node
rospy.init_node('telemetry_node')
rospy.log_info("Starting telemetry node")

# Initialize the transceiver object
transceiver = Transceiver(serial_num="/dev/ttyS0", freq=433, addr=0, power=22, rssi=False, air_speed=2400, relay=False)

# Publisher for uplink commands
uplink_publisher = rospy.Publisher('/uplink_commands', command_msg, queue_size=10)

# Function to receive data and publish to the topic
def main():
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        command = transceiver.receive_data()
        if command:
            rospy.loginfo(f"Received and publishing: component={command.component}, component_id={command.component_id}, command={command.command}")
            uplink_publisher.publish(command)
        rate.sleep()

if __name__ == '__main__':
    main()
