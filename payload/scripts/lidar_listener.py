#!/usr/bin/env python

import rospy
from payload.msg import lidar_raw_data_single
from payload.msg import found_debris_debugging
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt

class TestingError:
    def __init__(self):
        self._raw_data = np.zeros((8,8))
        self._blob_positions = np.array([0,0])
        self._sizes = []
        # self._start_plotting = True
        
        rospy.Subscriber('/raw_lidar_data_single', lidar_raw_data_single, self.callback)
        rospy.Subscriber('/found_debris', found_debris_debugging, self.callback_found)
        return




    def callback(self,raw_data):
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        print(f"Raw LiDAR Distances:\n{np.array(raw_data.distances_1).reshape(8,8)} \n")
        # print(f"LiDAR Label: {raw_data.label}\n")
        self._raw_data = np.array(raw_data.distances_1).reshape(8,8)
        return

    def callback_found(self,found_debris):
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        # print(f"Raw LiDAR Distances:\n{np.array(raw_data.distances_1).reshape(8,8)} \n")
        # print(f"LiDAR Label: {raw_data.label}\n")
        self._blob_positions = np.array(int(found_debris.blob_position[0]), int(found_debris.blob_position[1]))
        
        print("Size debris: ", found_debris.size, found_debris.num_pixels)
        print("blob position", self._blob_positions)


        plt.figure()
        plt.imshow(self._raw_data, cmap='viridis', interpolation= 'nearest')
        plt.colorbar(label='Value')
        plt.scatter(self._blob_positions[1], self._blob_positions[0], marker='x', color = 'r')
        plt.xlabel('Column')
        plt.ylabel('Row')
        plt.title('2D Plot of test_data')
        plt.savefig("lidar.png")
        return

    # def listener_func(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        
        # spin() simply keeps python from exiting until this node is stopped
     

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    test = TestingError()

    # test.listener_func()
    rospy.spin()
