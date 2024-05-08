#!/usr/bin/env python

import time
import rospy
import numpy

from std_msgs.msg import String
import vl53l5cx_ctypes as vl53l5cx

class Payload:
    def __init__(self):
        print("Uploading firmware, please wait...")
        self.vl53 = vl53l5cx.VL53L5CX()
        print("Done!")

        self.vl53.set_resolution(8 * 8)
        self.vl53.enable_motion_indicator(8 * 8)
        self.vl53.set_motion_distance(400, 1400)

        self.pub = rospy.Publisher('/raw_lidar_data', String, queue_size=10)

    def print_time(self, event=None):
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        #self.pub.publish(hello_str)

        #self.pub = rospy.Publisher('/raw_lidar_data', numpy_msg(Floats), queue_size=10)


if __name__ == '__main__':
    rospy.init_node("payload")
    myPayload = Payload()
    rospy.Timer(rospy.Duration(1.0/10.0), myPayload.print_time)
    rospy.spin()
