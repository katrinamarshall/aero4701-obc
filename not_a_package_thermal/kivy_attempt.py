import kivy
from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from kivy.uix.scrollview import ScrollView
import spidev
import time
import math

kivy.require('1.11.1')  # Replace '1.11.1' with your specific Kivy version number

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

# Constants
v_in = 3.3
R_ref = 10000

class TempApp(App):
    def build(self):
        self.layout = BoxLayout(orientation='vertical')
        self.labels = [Label(text='Initializing...', size_hint_y=None, height=40) for _ in range(5)]
        for label in self.labels:
            self.layout.add_widget(label)
        Clock.schedule_interval(self.update_temps, 5)  # Update every 5 seconds
        return self.layout

    def update_temps(self, dt):
        temps = [self.get_temp(i) for i in range(4)] + [self.get_pi_temp()]
        current_time = time.strftime("%H:%M:%S")
        for i, temp in enumerate(temps):
            self.labels[i].text = f'Channel {i}: {temp:.2f}°C' if i < 4 else f'Pi CPU: {temp:.2f}°C'
        # Optionally log to file here

    def read_adc(self, channel_num):
        if channel_num > 7 or channel_num < 0:
            return -1
        r = spi.xfer2([1, (8 + channel_num) << 4, 0])
        adc_out = ((r[1] & 3) << 8) + r[2]
        return adc_out

    def get_temp(self, adc_channel):
        raw_data = self.read_adc(adc_channel)
        voltage = (raw_data * v_in) / 1024
        R_thermistor = R_ref * ((v_in / voltage) - 1)
        a, b, c = 0.0010295, 0.000239255, 0.0000001558
        ln_r = math.log(R_thermistor)
        temp_k = 1 / (a + b * ln_r + c * ln_r ** 3)
        temp_c = temp_k - 273.15
        return temp_c

    def get_pi_temp(self):
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temp = f.read()
        return float(temp) / 1000

if __name__ == '__main__':
    TempApp().run()
