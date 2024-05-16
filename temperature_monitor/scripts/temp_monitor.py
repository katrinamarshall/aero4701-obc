#!/usr/bin/python
# -*- coding: utf-8 -*-
import spidev
import time
import math
from time import strftime

import rospy
from std_msgs.msg import String

#ROS SETUP
# Initialize the node
rospy.init_node('temperature_monitor_node', anonymous=True)
pub = rospy.Publisher('temperature_data', String, queue_size=10)
rate = rospy.Rate(0.2)  # 5 seconds

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000  # Ensure under 1.35 MHz

#Constants
v_in = 3.3  # Input reference voltage from Pi
R_ref = 10000  # Fixed reference resistor (10k ohm)

#File
file = open('/home/debra3/temperatures.csv', 'w')
#degrees celsius?
file.write('Time, Thermistor 0, Thermistor 1, Thermistor 2, Thermistor 3, Pi CPU \n') 


def readADC(channelNum):
    if channelNum > 7 or channelNum < 0:
        return -1
    r = spi.xfer2([1, (8 + channelNum) << 4, 0])
    adc_out = ((r[1] & 3) << 8) + r[2]
    return adc_out

def getTemp(adc_channel):
    raw_data = readADC(adc_channel)  # Read the ADC channel to get value
    voltage = (raw_data * v_in) / 1024  # Convert ADC raw data to voltage
    
    if voltage < 20:
        rospy.logwarn(f"Channel {adc_channel}: Voltage zero detected, possibly unconnected. Hardcoding temperature as 0°C.")
        return 0  # Hardcoded temperature for unconnected or faulty channels
    
    R_thermistor = R_ref * ((v_in / voltage) - 1)  # Calculate thermistor resistance using voltage divider formula

    # Steinhart-Hart coefficients for a typical 10k NTC thermistor
    a = 0.0010295
    b = 0.000239255
    c = 0.0000001558
    # a = 0.001129148
    # b = 0.000234125
    # c = 0.0000000876741
    ln_r = math.log(R_thermistor)  # Ensure R_thermistor > zero before taking log
    t1 = b * ln_r
    t2 = c * ln_r ** 3
    temp_k = 1 / (a + t1 + t2)  # Calculate temperature in Kelvin
    temp_c = temp_k - 273.15  # Convert Kelvin to Celsius

    # print(f"Channel {adc_channel}: {temp_c:.1f} °C \n {raw_data}/1023 => {voltage:.3f} V => {R_thermistor:.1f} Ω => {temp_k:.1f} K =>  ")
    print(f"Channel {adc_channel} Temp: {temp_c:.1f} °C (raw data - {raw_data})")

    return temp_c

def getPiTemp():
    with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
        temp = f.read()
    cpu_temp = float(temp) / 1000  # Convert millidegree to degree
    print(f"Pi Temp: {cpu_temp:.0f}°C")  # Print the CPU temperature formatted
    return cpu_temp

while not rospy.is_shutdown():
    temps = [getTemp(i) for i in range(4)]
    pi_temp = getPiTemp()
    current_time = strftime("%H:%M")
    message = f"{current_time},{','.join(f'{t:.2f}' for t in temps)},{pi_temp:.2f}"
    rospy.loginfo(message)  # Log msg to ROS
    pub.publish(message)  # Publish msg
    file.write(f"{message}\n")
    file.flush()
    rate.sleep()