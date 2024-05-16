#!/usr/bin/env python

import time
import rospy

from ina219 import INA219
from ina219 import DeviceRangeError
from eps.msg import current_voltage

SHUNT_OHMS = 0.1

class EPS:
    def __init__(self):
        self.pub = rospy.Publisher('/current_voltage', current_voltage, queue_size=10)

        # Initialise ina219s
        self.curr_volt_sensor_40 = INA219(SHUNT_OHMS, address=0x40)
        self.curr_volt_sensor_41 = INA219(SHUNT_OHMS, address=0x41)
        self.curr_volt_sensor_44 = INA219(SHUNT_OHMS, address=0x44)

        self.curr_volt_sensor_40.configure()
        self.curr_volt_sensor_41.configure()
        self.curr_volt_sensor_44.configure()
 
    def get_curr_volt(self, event=None):
        msg = current_voltage()

        self.curr_volt_sensor_40.wake()
        self.curr_volt_sensor_41.wake()
        self.curr_volt_sensor_44.wake()

        msg.voltage_40 = self.curr_volt_sensor_40.voltage()
        msg.current_40 = self.curr_volt_sensor_40.current()

        msg.voltage_41 = self.curr_volt_sensor_41.voltage()
        msg.current_41 = self.curr_volt_sensor_41.current()

        msg.voltage_44 = self.curr_volt_sensor_44.voltage()
        msg.current_44 = self.curr_volt_sensor_44.current()

        self.curr_volt_sensor_40.sleep()
        self.curr_volt_sensor_41.sleep()
        self.curr_volt_sensor_44.sleep()

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("eps")
    myEPS = EPS()
    rospy.Timer(rospy.Duration(1.0/2.0), myEPS.get_curr_volt)
    rospy.spin()
    