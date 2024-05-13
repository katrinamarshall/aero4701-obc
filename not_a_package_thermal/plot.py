#!/usr/bin/python
# -*- coding: utf-8 -*-
import spidev
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000  # Ensure under 1.35 MHz

# Constants
v_in = 3.3  # Input reference voltage from Pi
R_ref = 10000  # Fixed reference resistor (10k ohm)

# File for logging temperatures
file = open('temperatures.csv', 'w')
file.write('Elapsed Time (s), Thermistor 0, Thermistor 1, Thermistor 2, Thermistor 3, Pi CPU \n')

# Initialize plot
plt.style.use('fivethirtyeight')
fig, ax = plt.subplots()
x_data, y_data = [], []
lines = []
for _ in range(5):  # Five lines for four thermistors + CPU temp
    line, = ax.plot([], [], label=f'Thermistor {_}' if _ < 4 else 'Pi CPU')
    lines.append(line)
plt.legend()

def init():
    ax.set_xlim(0, 10)
    ax.set_ylim(20, 40)  # Adjust these limits based on expected temperature range
    return lines

start_time = time.time()

def update(frame):
    elapsed_time = time.time() - start_time  # Time in seconds since script start
    temps = [getTemp(i) for i in range(4)] + [getPiTemp()]
    x_data.append(elapsed_time)
    file.write(f"{elapsed_time:.2f},{','.join(f'{t:.2f}' for t in temps)}\n")
    file.flush()
    # Update data for each line
    for line, temp in zip(lines, temps):
        y_vals = line.get_ydata()
        y_vals.append(temp)
        line.set_data(x_data, y_vals)
    ax.relim()
    ax.autoscale_view()
    return lines

ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=5000, cache_frame_data=False)

plt.show()
