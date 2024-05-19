import RPi.GPIO as GPIO
import serial
import time

"""The SX126x class is used for interfacing with LoRa hat transceivers like the SX1268"""
class SX126x:
    # Pins on the transceiver
    M0 = 22
    M1 = 27
    # LoRa register settings. If the header is 0xC0, the settings persist when powered off.
    cfg_reg = [0xC2, 0x00, 0x09, 0x00, 0x00, 0x00, 0x62, 0x00, 0x12, 0x43, 0x00, 0x00]
    get_reg = bytes(12)
    rssi = False
    addr = 65535
    serial_n = ""
    addr_temp = 0

    # Start frequency of the LoRa module (in MHz)
    start_freq = 850

    # Offset between start and end frequency of the LoRa module (in MHz)
    offset_freq = 18

    SX126X_UART_BAUDRATE_1200 = 0x00
    SX126X_UART_BAUDRATE_2400 = 0x20
    SX126X_UART_BAUDRATE_4800 = 0x40
    SX126X_UART_BAUDRATE_9600 = 0x60
    SX126X_UART_BAUDRATE_19200 = 0x80
    SX126X_UART_BAUDRATE_38400 = 0xA0
    SX126X_UART_BAUDRATE_57600 = 0xC0
    SX126X_UART_BAUDRATE_115200 = 0xE0

    SX126X_PACKAGE_SIZE_240_BYTE = 0x00
    SX126X_PACKAGE_SIZE_128_BYTE = 0x40
    SX126X_PACKAGE_SIZE_64_BYTE = 0x80
    SX126X_PACKAGE_SIZE_32_BYTE = 0xC0

    SX126X_Power_22dBm = 0x00
    SX126X_Power_17dBm = 0x01
    SX126X_Power_13dBm = 0x02
    SX126X_Power_10dBm = 0x03

    lora_air_speed_dic = {
        1200: 0x01,
        2400: 0x02,
        4800: 0x03,
        9600: 0x04,
        19200: 0x05,
        38400: 0x06,
        62500: 0x07
    }

    lora_power_dic = {
        22: 0x00,
        17: 0x01,
        13: 0x02,
        10: 0x03
    }

    lora_buffer_size_dic = {
        240: SX126X_PACKAGE_SIZE_240_BYTE,
        128: SX126X_PACKAGE_SIZE_128_BYTE,
        64: SX126X_PACKAGE_SIZE_64_BYTE,
        32: SX126X_PACKAGE_SIZE_32_BYTE
    }

    def __init__(self, serial_num, freq, addr, power, rssi, air_speed=2400, net_id=0, buffer_size=240, crypt=0, relay=False, lbt=False, wor=False):
        self.rssi = rssi
        self.addr = addr
        self.freq = freq
        self.serial_n = serial_num
        self.power = power
        # Initialize the GPIO for M0 and M1 pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.M0, GPIO.OUT)
        GPIO.setup(self.M1, GPIO.OUT)
        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.HIGH)

        # The hardware UART of Pi3B+, Pi4B is /dev/ttyS0
        self.ser = serial.Serial(serial_num, 9600)
        self.ser.flushInput()
        self.set(freq, addr, power, rssi, air_speed, net_id, buffer_size, crypt, relay, lbt, wor)

    def set(self, freq, addr, power, rssi, air_speed=2400, net_id=0, buffer_size=240, crypt=0, relay=False, lbt=False, wor=False):
        self.send_to = addr
        self.addr = addr
        # Pull up the M1 pin when setting the module
        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.HIGH)
        time.sleep(0.1)

        low_addr = addr & 0xff
        high_addr = addr >> 8 & 0xff
        net_id_temp = net_id & 0xff
        if freq > 850:
            freq_temp = freq - 850
            self.start_freq = 850
            self.offset_freq = freq_temp
        elif freq > 410:
            freq_temp = freq - 410
            self.start_freq = 410
            self.offset_freq = freq_temp

        air_speed_temp = self.lora_air_speed_dic.get(air_speed, None)
        buffer_size_temp = self.lora_buffer_size_dic.get(buffer_size, None)
        power_temp = self.lora_power_dic.get(power, None)

        if rssi:
            rssi_temp = 0x80  # Enable RSSI value print
        else:
            rssi_temp = 0x00  # Disable RSSI value print

        l_crypt = crypt & 0xff
        h_crypt = crypt >> 8 & 0xff

        if relay == False:
            self.cfg_reg[3] = high_addr
            self.cfg_reg[4] = low_addr
            self.cfg_reg[5] = net_id_temp
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            self.cfg_reg[9] = 0x43 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        else:
            self.cfg_reg[3] = 0x01
            self.cfg_reg[4] = 0x02
            self.cfg_reg[5] = 0x03
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            self.cfg_reg[9] = 0x03 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        self.ser.flushInput()

        for i in range(2):
            self.ser.write(bytes(self.cfg_reg))
            r_buff = 0
            time.sleep(0.2)
            if self.ser.inWaiting() > 0:
                time.sleep(0.1)
                r_buff = self.ser.read(self.ser.inWaiting())
                if r_buff[0] == 0xC1:
                    pass
                else:
                    pass
                break
            else:
                print("Setting failed, setting again")
                self.ser.flushInput()
                time.sleep(0.2)
                print('\x1b[1A', end='\r')
                if i == 1:
                    print("Setting failed, press Esc to exit and run again")

        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.LOW)
        time.sleep(0.1)

    def get_settings(self):
        # The M1 pin of LoRa HAT must be high when entering setting mode and getting parameters
        GPIO.output(self.M1, GPIO.HIGH)
        time.sleep(0.1)

        # Send command to get setting parameters
        self.ser.write(bytes([0xC1, 0x00, 0x09]))
        if self.ser.inWaiting() > 0:
            time.sleep(0.1)
            self.get_reg = self.ser.read(self.ser.inWaiting())

        # Check the return characters from HAT and print the setting parameters
        if self.get_reg[0] == 0xC1 and self.get_reg[2] == 0x09:
            fre_temp = self.get_reg[8]
            addr_temp = self.get_reg[3] + self.get_reg[4]
            air_speed_temp = self.get_reg[6] & 0x03
            power_temp = self.get_reg[7] & 0x03

            print("Frequency is {0}.125MHz.".format(fre_temp))
            print("Node address is {0}.".format(addr_temp))
            print("Air speed is {0} bps".format(self.lora_air_speed_dic.get(air_speed_temp, None)))
            print("Power is {0} dBm".format(self.lora_power_dic.get(power_temp, None)))
            GPIO.output(self.M1, GPIO.LOW)

    def send(self, data):
        # Set the module to transmission mode
        GPIO.output(self.M1, GPIO.LOW)
        GPIO.output(self.M0, GPIO.LOW)
        time.sleep(0.1)

        # Send data
        self.ser.write(data)
        time.sleep(0.1)

    def receive(self):
        # Check if there is data to read
        if self.ser.inWaiting() > 0:
            time.sleep(0.5)
            r_buff = self.ser.read(self.ser.inWaiting())
            message = r_buff[3:]

            if self.rssi:
                print("The packet RSSI value: -{0}dBm".format(256 - r_buff[-1:][0]))
                self.get_channel_rssi()
            return message
        else:
            return None

    def get_channel_rssi(self):
        # Set the module to normal mode and get channel RSSI
        GPIO.output(self.M1, GPIO.LOW)
        GPIO.output(self.M0, GPIO.LOW)
        time.sleep(0.1)
        self.ser.flushInput()
        self.ser.write(bytes([0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02]))
        time.sleep(0.5)
        re_temp = bytes(5)
        if self.ser.inWaiting() > 0:
            time.sleep(0.1)
            re_temp = self.ser.read(self.ser.inWaiting())
        if re_temp[0] == 0xC1 and re_temp[1] == 0x00 and re_temp[2] == 0x02:
            print("The current noise RSSI value: -{0}dBm".format(256 - re_temp[3]))
        else:
            print("Failed to receive RSSI value")
