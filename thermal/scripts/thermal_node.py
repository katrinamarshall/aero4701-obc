#!/usr/bin/env python

import time
import rospy
import math

import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

from std_msgs.msg import String
from thermal.msg import temperatures

VIN = 3.3  # Input voltage
R_REF = 10000  # Fixed reference resistor (10k ohm)

class Thermal:
    def __init__(self):

        spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
        cs = digitalio.DigitalInOut(board.D5)
        self.mcp = MCP.MCP3008(spi, cs)
        
        # Publishers
        self.pub = rospy.Publisher('/temperatures', temperatures, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/operation_state', String, self.callback_state)

        # Initialise timer to take readings
        self.thermal_active = False
        self.mcp3008_reader = rospy.Timer(rospy.Duration(1.0/10.0), self.get_temps)

    # Callback for state changes
    def callback_state(self, state):
        
        # Check state
        if state.data == "Deorbit" or state.data == "Launch":
            self.thermal_active = False
        else:
            self.thermal_active = True
 
    # Take temperature readings
    def get_temps(self, event=None):
        if self.thermal_active:
            msg = temperatures()

            voltage1 = AnalogIn(self.mcp, MCP.P0).voltage
            voltage2 = AnalogIn(self.mcp, MCP.P2).voltage

            msg.thermistor_1 = self.voltage2temp(voltage1)
            msg.thermistor_2 = self.voltage2temp(voltage2)
            msg.pi = self.get_pi_temp()

            self.pub.publish(msg)

    def voltage2temp(self, voltage):

        if voltage < 0.01:
            rospy.logwarn(f"Voltage zero detected, possibly unconnected.")
            return 0  # Hardcoded temperature for unconnected or faulty channels

        # Steinhart-Hart coefficients for a typical 10k NTC thermistor
        a = 0.0010295
        b = 0.000239255
        c = 0.0000001558

        R_thermistor = R_REF * ((VIN / voltage) - 1)

        ln_r = math.log(R_thermistor)  # Ensure R_thermistor > zero before taking log
        t1 = b * ln_r
        t2 = c * ln_r ** 3
        temp_k = 1 / (a + t1 + t2)  # Calculate temperature in Kelvin
        temp_c = temp_k - 273.15 - 20.0 # Convert Kelvin to Celsius

        return temp_c

    def get_pi_temp(self):
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temp = f.read()
        cpu_temp = float(temp) / 1000  # Convert millidegree to degree
        return cpu_temp

if __name__ == "__main__":
    rospy.init_node("thermal")
    myThermal = Thermal()
    rospy.spin()