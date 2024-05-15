# #!/usr/bin/env python
# import rospy
# from std_msgs.msg import String
# from telemetry.msg import command_msg
# from transceivers import Transceiver
# from AX25UI import AX25UIFrame
# from debra.msg import payload_data

# # Callback to receive downlink data instructions
# def downlink_data_callback(data: String, transceiver: Transceiver):
#     info = data.data  
#     ssid_type = 0b1110 

#     ax25_frame = AX25UIFrame(info, ssid_type)
#     frame = ax25_frame.create_frame()

#     transceiver.send_deal(frame)

#     # transceiver.send_deal(data.data)

# def payload_data_callback(data: payload_data, transceiver: Transceiver):
#     # Initialise data and science ssid
#     info = (
#         f"{data.debris_position_x}{data.debris_position_y}{data.debris_position_z}{data.debris_velocity_x}{data.debris_velocity_y}{data.debris_velocity_z}{data.debris_diameter}{data.time_of_detection}{data.object_count}"
#     )
#     print(f"Science data: {info}")
#     ssid_type = 0b1111
    
#     # Create an ax.25 UI frame
#     ax25_frame = AX25UIFrame(info, ssid_type)
#     frame = ax25_frame.create_frame()

#     # Send data
#     transceiver.send_deal(frame)

# # Function to receive data and publish to the topic
# def main():
#     # Initialize the ROS node
#     rospy.init_node('telemetry_node')
#     rospy.loginfo("Starting telemetry node")
#     rate = rospy.Rate(1)

#     # Initialize the transceiver object
#     transceiver = Transceiver(serial_num="/dev/ttyS0", freq=433, addr=0, power=22, rssi=False, air_speed=2400, relay=False)

#     # Publisher for uplink commands
#     uplink_publisher = rospy.Publisher('/uplink_commands', command_msg, queue_size=10)

#     # Subscriber for downlink commands
#     downlink_subscriber = rospy.Subscriber('/downlink_data', String, lambda data: downlink_data_callback(data, transceiver))

#     payload_subscriber = rospy.Subscriber('/payload_data', payload_data, lambda data: downlink_data_callback(data, transceiver))

#     # Spin node
#     while not rospy.is_shutdown():
#         command = transceiver.receive_data()
#         if command:
#             rospy.loginfo(f"Received and publishing: component={command.component}, component_id={command.component_id}, command={command.command}")
#             uplink_publisher.publish(command)
#         rate.sleep()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from telemetry.msg import command_msg
from transceivers import Transceiver
from AX25UI import AX25UIFrame
from debra.msg import payload_data

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

    def downlink_data_callback(self, data):
        info = data.data  
        ssid_type = 0b1110

        ax25_frame = AX25UIFrame(info, ssid_type)
        frame = ax25_frame.create_frame()

        self.transceiver.send_deal(frame)

    def payload_data_callback(self, data):
        # Initialise data and science ssid
        info = (
            f"{data.debris_position_x}\n"
            f"{data.debris_position_y}\n"
            f"{data.debris_position_z}\n"
            f"{data.debris_velocity_x}\n"
            f"{data.debris_velocity_y}\n"
            f"{data.debris_velocity_z}\n"
            f"{data.debris_diameter}\n"
            f"{data.time_of_detection}\n"
            f"{data.object_count}"
        )
        print(f"Science data: {info}")
        ssid_type = 0b1111

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

if __name__ == '__main__':
    telemetry = Telemetry()
    telemetry.run()
