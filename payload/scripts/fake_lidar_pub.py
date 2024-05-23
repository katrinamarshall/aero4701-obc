#!/usr/bin/env python

import time
import rospy
import numpy as np
import random

from std_msgs.msg import String
from payload.msg import lidar_raw_data_single
#from lidar_listener import listener_func

class FakeLidar:
    def __init__(self, label):
        # print("Sending lidar data...")
        self.label = label
        
       

        self.pub = rospy.Publisher('/raw_lidar_data_single', lidar_raw_data_single, queue_size=10)
        self.ready = True
        # print("Done!")


    def get_lidar_data(self, event=None):
        if True:
            # data  = np.array([[40, 30, 50, 4000, 100, 140, 4000, 4000], 
            #                         [10, 10, 1000, 4000, 4000, 4000, 4000, 4000],
            #                         [10, 4000, 1000, 1000, 1000, 4000, 4000, 4000],
            #                         [1000, 1040, 1000, 4000, 4000, 4000, 4000, 4000],
            #                         [4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000],
            #                         [4000, 4000, 4000, 4000, 4000, 1500, 1540, 1580], 
            #                         [4000, 2000, 4000, 4000, 4000, 4000, 4000, 4000], 
            #                         [4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000]])
            
            data = np.full((8,8), 4000)
            row = random.randint(0,6)
            col = random.randint(0,6)
            data[row:row+2, col:col+2] = 50
     
            # 2d array of motion data (always 4x4?)
            #motion = numpy.flipud(numpy.array(data.motion_indicator.motion[0:16]).reshape((4, 4)))
            # 2d array of distance
            distance = data.flatten() # numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
            #print(f"Distance: {distance} \n")
            self.pub.publish( distance)
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
    rospy.init_node("fake_lidar")
    myFakeLidar1 = FakeLidar(1)
    # myFakeLidar2 = FakeLidar(2)
    # myFakeLidar3 = FakeLidar(3)
    # myFakeLidar4 = FakeLidar(4)

    rospy.Timer(rospy.Duration(1.0/2.0), myFakeLidar1.get_lidar_data)
    # rospy.Timer(rospy.Duration(1.0/2.0), myFakeLidar2.get_lidar_data)
    # rospy.Timer(rospy.Duration(1.0/2.0), myFakeLidar3.get_lidar_data)
    # rospy.Timer(rospy.Duration(1.0/2.0), myFakeLidar4.get_lidar_data)

    rospy.spin()
