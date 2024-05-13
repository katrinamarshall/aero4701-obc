#!/usr/bin/env python

import time
import rospy
import numpy

from std_msgs.msg import String
from payload.msg import lidar_raw_data
#from lidar_listener import listener_func
import vl53l5cx_ctypes as vl53l5cx

class Payload:
    def __init__(self):
        print("Uploading firmware, please wait...")
        self.vl53 = vl53l5cx.VL53L5CX()
        print("Done!")

        self.vl53.set_resolution(8 * 8)
        self.vl53.enable_motion_indicator(8 * 8)
        self.vl53.set_motion_distance(400, 1400)
        self.vl53.start_ranging()

        self.pub = rospy.Publisher('/raw_lidar_data', lidar_raw_data, queue_size=10)

    def get_lidar_data(self, event=None):
        if self.vl53.data_ready():
            data = self.vl53.get_data()
            # 2d array of motion data (always 4x4?)
            #motion = numpy.flipud(numpy.array(data.motion_indicator.motion[0:16]).reshape((4, 4)))
            # 2d array of distance
            distance = numpy.array(data.distance_mm).flatten() # numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
            #print(f"Distance: {distance} \n")
            self.pub.publish(distance)
            # 2d array of reflectance
            # reflectance = numpy.flipud(numpy.array(data.reflectance).reshape((8, 8)))
            # 2d array of good ranging data
            #status = numpy.isin(numpy.flipud(numpy.array(data.target_status).reshape((8, 8))), (STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE))
            #print(f"Motion: {motion}, \n Distance: {distance}, \n Reflectance: {reflectance}, \n Status: {status}\n\n")
        time.sleep(0.1)

        #rospy.loginfo(hello_str)
        #self.pub.publish(hello_str)

        #self.pub = rospy.Publisher('/raw_lidar_data', numpy_msg(Floats), queue_size=10)


if __name__ == '__main__':
    rospy.init_node("payload")
    myPayload = Payload()
    rospy.Timer(rospy.Duration(1.0/2.0), myPayload.get_lidar_data)
    rospy.spin()
