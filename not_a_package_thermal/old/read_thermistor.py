#!/usr/bin/python
# -*- coding: utf-8 -*-
import spidev
import time
import math
from time import strftime

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

v_in = 3.3  # input voltage from Raspberry Pi
R = 10000  # Fixed resistor value (10k ohm)

file = open('temp_test.txt', 'w')
file.write('Time,Temperature ADC 0,Temperature ADC 1\n')

def read_adc_channel(channel_num):
    if channel_num > 7 or channel_num < 0:
        return -1
    r = spi.xfer2([1, 8 + channel_num << 4, 0])
    adcout = ((r[1] & 3) << 8) + r[2]
    return adcout

def temp_get(adc):
    value = read_adc_channel(adc)  # read the adc value
    volts = (value * v_in) / 1024  # convert adc value to voltage
    R_thermistor = R * ((v_in / volts) - 1)  # calculate thermistor resistance using voltage divider formula

    # Steinhart-Hart coefficients for a typical 10k NTC thermistor
    # These need to be adjusted according to the thermistor's datasheet
    a = 0.001129148
    b = 0.000234125
    c = 0.0000000876741

    lnr = math.log(R_thermistor)
    t1 = b * lnr
    t2 = c * lnr ** 3
    temp_k = 1 / (a + t1 + t2)  # calculate temperature in Kelvin

    temp_c = temp_k - 273.15  # convert Kelvin to Celsius

    print(f"{value}/1023 => {volts:.3f} V => {R_thermistor:.1f} Ω => {temp_k:.1f} K => {temp_c:.1f} °C from adc {adc}")
    return temp_c

while True:
    # Get temperatures from ADC channels 0 and 1
    temp_adc0 = temp_get(0)
    temp_adc1 = temp_get(1)
    current_time = strftime("%H:%M")
    file.write(f"{current_time},{temp_adc0:.2f},{temp_adc1:.2f}\n")
    file.flush()  # Ensure data is written to the file
    time.sleep(10)
