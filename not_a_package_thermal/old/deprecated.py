#!/usr/bin/python
# -*- coding: utf-8 -*-
# A simple script to log data from the MCP3208
# Based off mcp3208.py

import spidev
import time
import math
from time import strftime
import string

using BaremetalPi

init_spi("/dev/spidev0.0", max_speed_hz = 1000)
tx_buf = [0x01, 0x80, 0x00]
rx_buf = zeros(UInt8, 3)

ret = spi_transfer!(1, tx_buf, rx_buf)
V = ( (UInt16(rx_buf[2] & 3) << 8) + rx_buf[3] )*3.3/1024
R_div = 10_000         # ............. Resistor value at the voltage divider [Ω]
th_R = R_div*(3.3/V - 1)

th_β  = 3380           # ................ Beta coefficient of the thermistor [K]
th_R₀ = 10_000         # ............. Reference thermistor resistance at T₀ [Ω]
th_T₀ = 25             # ....... Reference temperature T₀ of the thermistor [°C]

T = 1/( log(th_R / th_R₀)/th_β + 1/(th_T₀ + 273.15) ) - 273.15

using Dates
using Printf
using BaremetalPi

################################################################################
#                                Configuration
################################################################################

const R_div = 10_000   # ............. Resistor value at the voltage divider [Ω]
const th_β  = 3380     # ................ Beta coefficient of the thermistor [K]
const th_R₀ = 10_000   # ............. Reference thermistor resistance at T₀ [Ω]
const th_T₀ = 25       # ....... Reference temperature T₀ of the thermistor [°C]

################################################################################
#                                  Functions
################################################################################

function acquire_temperature()
    # Acquire the AD measurement
    # ==========================================================================

    tx_buf = [0x01, 0x80, 0x00]
    rx_buf = zeros(UInt8, 3)
    V      = 0

    ret = spi_transfer!(1, tx_buf, rx_buf)

    if ret != 3
        @warn("The data received from MCP3008 does not have 3 bytes.")
        return NaN
    end

    # AD measurement.
    V = ((UInt16(rx_buf[2] & 3) << 8) + rx_buf[3]) * 3.3 / 1024

    if V == 0
        @warn("The MCP3008 measured 0V.")
        return NaN
    end

    if V > 3.25
        @warn("The MCP3008 measured a too high voltage (> 3.25V).")
        return NaN
    end

    # Convert the measurement to temperature
    # ==========================================================================

    th_R = R_div * (3.3 / V - 1)
    T    = 1 / (log(th_R / th_R₀) / th_β + 1 / (th_T₀ + 273.15)) - 273.15

    return T
end

function run()
    # Let's sample at 1kHz to improve the accuracy of MCP3008.
    init_spi("/dev/spidev0.0", max_speed_hz = 1000)

    while true
        T = acquire_temperature()

        if !isnan(T)
            @printf("%-30s %3.2f\n", string(now()), T)
        else
            println("[ERROR] Problem when acquiring AD measurement.")
        end
        sleep(5)
    end
end

run()
