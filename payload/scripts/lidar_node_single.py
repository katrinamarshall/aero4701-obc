#!/usr/bin/env python

import time
import rospy
import numpy
import RPi.GPIO as GPIO

from std_msgs.msg import String
from payload.msg import lidar_raw_data_single
import vl53l5cx_ctypes as vl53l5cx

# LiDAR pinout for Em:
# VDD - ignore
# VIN - 3.3V
# GND - GND (make sure gnd of pi and power supply to LiDAR are connected)
# SDA - SDA of Pi
# SCL - SCL of Pi
# LPn - pin 31 of Pi (this pin is needed to set i2c address)

# Debugging: check if the LiDAR is detected on the i2c bus using `i2cdetect -y 1` in command line

LPN_1 = 6

I2C_ADD_1 = 0x56

RANGING_FREQ = 10 # (Hz) For 8x8 ranging frequency must be between 1-15Hz

class Lidar:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LPN_1, GPIO.OUT)

        print("Uploading firmware, please wait...")
        GPIO.output(LPN_1, 0)
        time.sleep(0.5)

        GPIO.output(LPN_1, 1)
        try:
            self.vl53_1 = vl53l5cx.VL53L5CX(i2c_addr=I2C_ADD_1, skip_init=True)
        except:
            self.vl53_1 = vl53l5cx.VL53L5CX(skip_init=True)
            self.vl53_1.set_i2c_address(I2C_ADD_1)

        print("Addresses set!")

        self.vl53_1.init()
        self.vl53_1.set_resolution(8*8)
        self.vl53_1.set_ranging_frequency_hz(RANGING_FREQ)
        self.vl53_1.set_sharpener_percent(70)
        self.vl53_1.start_ranging()

        print("Done!")

        # Publishers
        self.pub = rospy.Publisher('/raw_lidar_data_single', lidar_raw_data_single, queue_size=10)

       # Initialise timer to take readings
        self.lidar_active = False
        self.vl53l5cx_reader = rospy.Timer(rospy.Duration(1.0/10.0), self.get_lidar_data)

       # Subscribers
        rospy.Subscriber('/operation_state', String, self.callback_state)


    # Callback for state changes
    def callback_state(self, state):
        
        # Check state
        if state.data == "Debris Detection":
            self.lidar_active = True
        else:
            self.lidar_active = False

    def get_lidar_data(self, event=None):
        if self.lidar_active:
            msg = lidar_raw_data_single()

            if self.vl53_1.data_ready():
                data1 = self.vl53_1.get_data()
                msg.distances_1 = numpy.array(data1.distance_mm).flatten() # numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
                msg.status_1 = numpy.array(data1.target_status).flatten()   
                self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("lidar")
    myLidar = Lidar()
    # rospy.Timer(rospy.Duration(1.0/10.0), myLidar.get_lidar_data)
    rospy.spin()
