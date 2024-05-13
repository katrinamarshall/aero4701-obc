from time import sleep

class Thermistor:
    def __init__(self, pin):
        self.thermistor = ADC(pin)
        
    def ReadTemperature(self):
        # Get Voltage value from ADC    
        adc_value = self.thermistor.read_u16()
        Vout = (3.3/65535)*adc_value
        
        # Voltage Divider
        Vin = 3.3
        Ro = 10000  # 10k Resistor

        # Steinhart Constants
        A = 0.001129148
        B = 0.000234125
        C = 0.0000000876741

        # Calculate Resistance
        Rt = (Vout * Ro) / (Vin - Vout) 
    
        # Steinhart - Hart Equation
        TempK = 1 / (A + (B * math.log(Rt)) + C * math.pow(math.log(Rt), 3))

        # Convert from Kelvin to Celsius
        TempC = TempK - 273.15

        return round(TempC,2)

adcpin = 26
thermistor = Thermistor(adcpin)

while True:
    TempC = thermistor.ReadTemperature()
    print(TempC)
    sleep(5)
#https://www.halvorsen.blog/documents/technology/iot/pico/pico_thermisor.php/raspberry_pi.php