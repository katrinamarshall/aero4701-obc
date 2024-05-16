import rospy
from adcs.msg import imu_data_packet
import numpy as np

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    print(f"Acceleration (m/s^2):\n{np.array(data.acceleration)} \n")
    print(f"Gyroscope (rad/s):\n{np.array(data.gyro)} \n")
    print(f"Magnetometer (microteslas):\n{np.array(data.magnetometer)} \n")

def listener_func():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/imu_data', imu_data_packet, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener_func()