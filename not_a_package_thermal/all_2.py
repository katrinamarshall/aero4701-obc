#!/usr/bin/python
# -*- coding: utf-8 -*-
import spidev
import time
import math
from time import strftime
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Setup SPI
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

v_in = 3.3  # Input voltage from Raspberry Pi
R = 10000  # Fixed resistor value (10k ohm)

file = open('temperatures.csv', 'w')
file.write('Time,Thermistor 0,Thermistor 1,Thermistor 2,Thermistor 3\n')

# Prepare matplotlib
fig, ax = plt.subplots()
times = []
temps = [[], [], [], []]
lines = [ax.plot([], [], label=f'Thermistor {i}')[0] for i in range(4)]
ax.set_xlabel('Time')
ax.set_ylabel('Temperature (°C)')
plt.legend()

def init():
    ax.set_xlim(0, 50)  # Adjust based on your need
    ax.set_ylim(10, 50)  # Adjust based on expected temperature range
    return lines

def readADC(channelNum):
    if channelNum > 7 or channelNum < 0:
        return -1
    r = spi.xfer2([1, (8 + channelNum) << 4, 0])
    adcout = ((r[1] & 3) << 8) + r[2]
    return adcout

def temp_get(adc):
    value = readADC(adc)
    volts = (value * v_in) / 1024
    R_thermistor = R * ((v_in / volts) - 1)
    lnr = math.log(R_thermistor)
    a = 0.0010295
    b = 0.000239255
    c = 0.0000001558
    t1 = b * lnr
    t2 = c * lnr ** 3
    temp_k = 1 / (a + t1 + t2)
    temp_c = temp_k - 273.15
    return temp_c

def update(frame):
    current_time = strftime("%H:%M:%S")
    times.append(current_time)
    new_temps = [temp_get(i) for i in range(4)]
    for i in range(4):
        temps[i].append(new_temps[i])
        lines[i].set_data(times, temps[i])
    if len(times) > 50:  # Adjust this to control data window
        ax.set_xlim(times[-50], times[-1])
    file.write(f"{current_time},{new_temps[0]:.2f},{new_temps[1]:.2f},{new_temps[2]:.2f},{new_temps[3]:.2f}\n")
    file.flush()
    return lines

ani = FuncAnimation(fig, update,
cache_frame_data=False, init_func=init, blit=True, interval=10000)
plt.show()
