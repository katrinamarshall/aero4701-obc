#!/usr/bin/env python

import time
import rospy

from std_msgs.msg import String

from ina219 import INA219
from ina219 import DeviceRangeError
from eps.msg import current_voltage

from numpy import random

SHUNT_OHMS = 0.1

class EPS:
    def __init__(self):

        # Publishers
        self.pub = rospy.Publisher('/current_voltage', current_voltage, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/operation_state', String, self.callback_state)

        # Initialise INA219s
        self.curr_volt_sensor_40 = INA219(SHUNT_OHMS, address=0x40)
        self.curr_volt_sensor_41 = INA219(SHUNT_OHMS, address=0x41)
        self.curr_volt_sensor_44 = INA219(SHUNT_OHMS, address=0x44)

        self.curr_volt_sensor_40.configure()
        self.curr_volt_sensor_41.configure()
        self.curr_volt_sensor_44.configure()

        # Initialise timer to take readings
        self.eps_active = False
        self.ina219_reader = rospy.Timer(rospy.Duration(1.0/10.0), self.get_curr_volt)

    # Callback for state changes
    def callback_state(self, state):
        
        # Check state
        if state.data == "Deorbit" or state.data == "Launch":
            self.eps_active = False
        else:
            self.eps_active = True
 
    # Take current voltage readings
    def get_curr_volt(self, event=None):
        if self.eps_active:
            msg = current_voltage()

            self.curr_volt_sensor_40.wake()
            self.curr_volt_sensor_41.wake()
            self.curr_volt_sensor_44.wake()

            msg.voltage_40 = 3.24 + random.rand() #self.curr_volt_sensor_40.voltage()
            msg.current_40 = self.curr_volt_sensor_40.current()

            msg.voltage_41 = 4.66 + random.rand() #self.curr_volt_sensor_41.voltage()
            msg.current_41 = self.curr_volt_sensor_41.current()

            msg.voltage_44 = 2.77 + random.rand() #self.curr_volt_sensor_44.voltage()
            msg.current_44 = self.curr_volt_sensor_44.current()

            self.curr_volt_sensor_40.sleep()
            self.curr_volt_sensor_41.sleep()
            self.curr_volt_sensor_44.sleep()

            self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("eps")
    myEPS = EPS()
    rospy.spin()
    