#!/usr/bin/env python
import rospy
from payload.msg import payload_data
from std_msgs.msg import String
import numpy as np

def callback(debris_packet_info):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    print(f"Data received:\n{debris_packet_info} \n")
    # print(f"LiDAR Label: {raw_data.label}\n")

def listener_func():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener_downlink', anonymous=True)
    print("Ready")

    rospy.Subscriber('/payload_data', payload_data, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener_func()