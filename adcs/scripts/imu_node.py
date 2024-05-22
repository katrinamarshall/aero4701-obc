#!/usr/bin/env python

import time
import rospy

from std_msgs.msg import String

import board
import adafruit_bno055
from adcs.msg import imu_data_packet

class IMU:
    def __init__(self):

        # Initialise timer to take readings
        imu_active = False
        self.bno055_reader = rospy.Timer(rospy.Duration(0.0), self.get_imu_data)

        # Publishers
        self.pub = rospy.Publisher('/imu_data', imu_data_packet, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/operation_state', String, self.callback_state)

        # Initialise imu
        self.i2c = board.I2C()
        self.bno055 = adafruit_bno055.BNO055_I2C(self.i2c)

    # Callback for state changes
    def callback_state(self, state):
        
        # Check state
        if state.data == "Deorbit" or state.data == "Launch":
            self.imu_active = False
        else:
            self.imu_active = True

        # Start/Stop reading IMU
        if self.imu_active:
            self.bno055_reader = rospy.Timer(rospy.Duration(1.0/10.0), self.get_imu_data)
        else:
            self.bno055_reader.shutdown() 
 
    def get_imu_data(self, event=None):
        msg = imu_data_packet()

        msg.acceleration = self.bno055.acceleration # Accelerometer (m/s^2)
        msg.gyro = self.bno055.gyro # Gyroscope (rad/sec)
        msg.magnetometer = self.bno055.magnetic # Magnetometer (microteslas)
        msg.temperature = self.bno055.temperature # Temperature (degrees C)

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("imu")
    myIMU = IMU()
    rospy.spin()
    