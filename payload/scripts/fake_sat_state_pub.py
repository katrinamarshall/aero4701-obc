
import time
from datetime import datetime
import rospy
import numpy as np


from payload.msg import sat_info


class FakeSat:
    def __init__(self):
        # print("Sending lidar data...")

        self.pub = rospy.Publisher('/sat_info',sat_info, queue_size=10)
        self.ready = True
        # print("Done!")


    def get_sat_data(self, event=None):
        while not rospy.is_shutdown():
            position = np.array([-9006.10175046, -6926679.95121315, 9565.3464146])# ECI pos
            velocity = np.array([4559.07205974, 12.98671741, 6062.13485532])#ECI vel
            attitude = np.array([1,0,0,0])#quaternion
            timestamp = float(datetime.now().microsecond) # ms

            self.pub.publish(position, velocity, attitude, timestamp)

        time.sleep(0.1)

        #rospy.loginfo(hello_str)time
        #self.pub.publish(hello_str)

        #self.pub = rospy.Publisher('/raw_lidar_data', numpy_msg(Floats), queue_size=10)


if __name__ == '__main__':
    rospy.init_node("fake_sat_state")
    myFakeSat = FakeSat()
    rospy.Timer(rospy.Duration(1.0/2.0), myFakeSat.get_sat_data())
    rospy.spin()
