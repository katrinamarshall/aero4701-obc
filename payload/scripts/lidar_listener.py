#!/usr/bin/env python

import rospy
from payload.msg import lidar_raw_data_single
from std_msgs.msg import String
import numpy as np

def callback(raw_data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    print(f"Raw LiDAR Distances:\n{np.array(raw_data.distances_1).reshape(8,8)} \n")
    # print(f"LiDAR Label: {raw_data.label}\n")

def listener_func():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/raw_lidar_data_single', lidar_raw_data_single, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener_func()