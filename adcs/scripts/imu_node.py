#!/usr/bin/env python

import time
import rospy

import board
import adafruit_bno055
from adcs.msg import imu_data_packet

class IMU:
    def __init__(self):
        self.pub = rospy.Publisher('/imu_data', imu_data_packet, queue_size=10)

        # Initialise imu
        self.i2c = board.I2C()
        self.bno055 = adafruit_bno055.BNO055_I2C(self.i2c)
 
    def get_imu_data(self, event=None):
        msg = imu_data_packet()

        print(self.bno055.acceleration)
        print(self.bno055.gyro)
        print(self.bno055.magnetic)
        print(self.bno055.temperature)

        # msg.acceleration = self.bno055.acceleration # Accelerometer (m/s^2)
        # msg.gyro = self.bno055.gyro # Gyroscope (rad/sec)
        # msg.magnetometer = self.bno055.magnetic # Magnetometer (microteslas)

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("imu")
    myIMU = IMU()
    rospy.Timer(rospy.Duration(1.0/10.0), myIMU.get_imu_data)
    rospy.spin()
    