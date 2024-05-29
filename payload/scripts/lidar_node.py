#!/usr/bin/env python

import time
import rospy
import numpy
import RPi.GPIO as GPIO

from std_msgs.msg import String
from payload.msg import lidar_raw_data
import vl53l5cx_ctypes as vl53l5cx

LPN_1 = 6
LPN_2 = 12
LPN_3 = 13
LPN_4 = 16

I2C_ADD_1 = 0x56
I2C_ADD_2 = 0x60
I2C_ADD_3 = 0x64
I2C_ADD_4 = 0x68

RANGING_FREQ = 10

class Lidar:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(LPN_1, GPIO.OUT)
        GPIO.setup(LPN_2, GPIO.OUT)
        GPIO.setup(LPN_3, GPIO.OUT)
        GPIO.setup(LPN_4, GPIO.OUT)

        print("Uploading firmware, please wait...")
        GPIO.output(LPN_1, 0)
        GPIO.output(LPN_2, 0)
        GPIO.output(LPN_3, 0)
        GPIO.output(LPN_4, 0)
        time.sleep(0.5)

        GPIO.output(LPN_1, 1)
        try:
            self.vl53_1 = vl53l5cx.VL53L5CX(i2c_addr=I2C_ADD_1, skip_init=True)
        except:
            self.vl53_1 = vl53l5cx.VL53L5CX(skip_init=True)
            self.vl53_1.set_i2c_address(I2C_ADD_1)

        GPIO.output(LPN_2, 1)
        try:
            self.vl53_2 = vl53l5cx.VL53L5CX(i2c_addr=I2C_ADD_2, skip_init=True)
        except:
            self.vl53_2 = vl53l5cx.VL53L5CX(skip_init=True)
            self.vl53_2.set_i2c_address(I2C_ADD_2)

        GPIO.output(LPN_3, 1)
        try:
            self.vl53_3 = vl53l5cx.VL53L5CX(i2c_addr=I2C_ADD_3, skip_init=True)
        except:
            self.vl53_3 = vl53l5cx.VL53L5CX(skip_init=True)
            self.vl53_3.set_i2c_address(I2C_ADD_3)

        GPIO.output(LPN_4, 1)
        try:
            self.vl53_4 = vl53l5cx.VL53L5CX(i2c_addr=I2C_ADD_4, skip_init=True)
        except:
            self.vl53_4 = vl53l5cx.VL53L5CX(skip_init=True)
            self.vl53_4.set_i2c_address(I2C_ADD_4)

        print("Addresses set!")

        self.vl53_1.init()
        self.vl53_1.set_resolution(8*8)
        self.vl53_1.set_ranging_frequency_hz(RANGING_FREQ)
        self.vl53_1.start_ranging()

        print("Lidar 1 initialised!")

        self.vl53_2.init()
        self.vl53_2.set_resolution(8*8)
        self.vl53_2.set_ranging_frequency_hz(RANGING_FREQ)
        self.vl53_2.start_ranging()

        print("Lidar 2 initialised!")

        self.vl53_3.init()
        self.vl53_3.set_resolution(8*8)
        self.vl53_3.set_ranging_frequency_hz(RANGING_FREQ)
        self.vl53_3.start_ranging()

        print("Lidar 3 initialised!")

        self.vl53_4.init()
        self.vl53_4.set_resolution(8*8)
        self.vl53_4.set_ranging_frequency_hz(RANGING_FREQ)
        self.vl53_4.start_ranging()

        print("Done!")

        # Initialise timer to take readings
        self.lidar_active = False
        self.vl53l5cx_reader = rospy.Timer(rospy.Duration(1.0/10.0), self.get_lidar_data)

        # Publishers
        self.pub = rospy.Publisher('/raw_lidar_data', lidar_raw_data, queue_size=5)
        
        # Subscribers
        rospy.Subscriber('/operation_state', String, self.callback_state)

    # Callback for state changes
    def callback_state(self, state):
        
        # Check state
        if state.data == "Debris Detection":
            self.lidar_active = True
        else:
            self.lidar_active = False
    
    # Get LiDAR raw data
    def get_lidar_data(self, event=None):
        if self.lidar_active:
            msg = lidar_raw_data()

            if self.vl53_1.data_ready() and self.vl53_2.data_ready() and self.vl53_3.data_ready() and self.vl53_4.data_ready():
                data1 = self.vl53_1.get_data()
                msg.distances_1 = numpy.array(data1.distance_mm).flatten() # numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
                msg.status_1 = numpy.array(data1.target_status).flatten()

                data2 = self.vl53_2.get_data()
                msg.distances_2 = numpy.array(data2.distance_mm).flatten() # numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
                msg.status_2 = numpy.array(data2.target_status).flatten()

                data3 = self.vl53_3.get_data()
                msg.distances_3 = numpy.array(data3.distance_mm).flatten() # numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
                msg.status_3 = numpy.array(data3.target_status).flatten()

                data4 = self.vl53_4.get_data()
                msg.distances_4 = numpy.array(data4.distance_mm).flatten() # numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
                msg.status_4 = numpy.array(data4.target_status).flatten()

                self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("payload")
    myLidar = Lidar()
    rospy.spin()