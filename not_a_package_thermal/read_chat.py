import spidev
import time
import math 

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

def read_channel(channel):
    val = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((val[1] & 3) << 8) + val[2]
    print("Raw ADC Data: ", data)  # Debug print to check raw data
    return data

def calculate_temperature(data, R_ref=10000.0, B=3950.0, T_ref=298.15):
    # Calculate the resistance of the thermistor
    R_thermistor = (R_ref * (1023.0 / data - 1.0))
    
    # Calculate the temperature in Kelvin
    temp_K = B / (math.log(R_thermistor / R_ref) + B / T_ref)
    
    # Convert Kelvin to Celsius
    return temp_K - 273.15

# Main loop
while True:
    data = read_channel(0)
    temperature = calculate_temperature(data)
    print("Temperature: {:.2f} C".format(temperature))
    time.sleep(1)
