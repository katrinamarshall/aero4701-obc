#!/usr/bin/env python

import time
import rospy

from std_msgs.msg import String

import board
import adafruit_bno055
from adcs.msg import imu_data_packet

class IMU:
    def __init__(self):

        # Publishers
        self.pub = rospy.Publisher('/imu_data', imu_data_packet, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/operation_state', String, self.callback_state)

        # Initialise imu
        self.i2c = board.I2C()
        self.bno055 = adafruit_bno055.BNO055_I2C(self.i2c)
        self.bno055.mode = adafruit_bno055.NDOF_MODE

        # Initialise timer to take readings
        self.imu_active = True
        self.bno055_reader = rospy.Timer(rospy.Duration(1.0), self.get_imu_data)

    # Callback for state changes
    def callback_state(self, state):
        
        # Check state
        if state.data == "Deorbit" or state.data == "Launch":
            self.imu_active = False
        else:
            self.imu_active = True
 
    def get_imu_data(self, event=None):
        if self.imu_active:
            msg = imu_data_packet()

            # msg.acceleration = self.bno055.acceleration # Accelerometer (m/s^2)
            # msg.gyro = self.bno055.gyro # Gyroscope (rad/sec)
            # msg.magnetometer = self.bno055.magnetic # Magnetometer (microteslas)
            msg.orientation = self.bno055.quaternion # Temperature (degrees C)

            self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("imu")
    myIMU = IMU()
    rospy.spin()
    