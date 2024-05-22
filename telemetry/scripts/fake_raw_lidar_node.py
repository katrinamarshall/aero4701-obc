#!/usr/bin/env python

import rospy
from debra.msg import raw_lidar_msg
import random

class FakeRawLidarNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('fake_raw_lidar_node')
        
        # Create a publisher for the raw_lidar topic
        self.publisher = rospy.Publisher('/raw_lidar', raw_lidar_msg, queue_size=10)
        
        # Set the rate at which to publish messages
        self.rate = rospy.Rate(0.1)  # 1 Hz

    def generate_fake_data(self):
        """Generate fake lidar data"""
        msg = raw_lidar_msg()
        msg.distances_1 = [random.randint(0, 1000) for _ in range(64)]
        msg.distances_2 = [random.randint(0, 1000) for _ in range(64)]
        msg.distances_3 = [random.randint(0, 1000) for _ in range(64)]
        msg.distances_4 = [random.randint(0, 1000) for _ in range(64)]
        return msg

    def run(self):
        """Main loop to publish fake data"""
        while not rospy.is_shutdown():
            fake_data = self.generate_fake_data()
            self.publisher.publish(fake_data)
            self.rate.sleep()

if __name__ == '__main__':
    node = FakeRawLidarNode()
    node.run()
