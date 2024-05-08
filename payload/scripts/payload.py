#!/usr/bin/env python

import time
import rospy

from std_msgs.msg import String

class Talker:
    def __init__(self):
        self.pub = rospy.Publisher('/chatter', String, queue_size=10)

    def print_time(self, event=None):
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        self.pub.publish(hello_str)

if __name__ == '__main__':
    rospy.init_node("talker")
    talker_obj = Talker()
    rospy.Timer(rospy.Duration(1.0/10.0), talker_obj.print_time)

    rospy.spin()
