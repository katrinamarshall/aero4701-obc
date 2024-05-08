#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
from src import sx1268 as sx126x
import select
import termios
import tty
from threading import Timer
import json
import rospy

# Define global variables for terminal settings (to see commands while writing)
old_settings = termios.tcgetattr(sys.stdin)

# File path of received commands for visualization
json_file_path = 'received_commands.json'

# Initialize the LoRa node at 433MHz
node = sx126x.sx126x(serial_num="/dev/ttyS0", freq=433, addr=0, power=22, rssi=False, air_speed=2400, relay=False)

def append_to_json(component, component_id, command):
    """ Append command data to a JSON file. """
    command_data = {
        "component": component,
        "component_id": component_id,
        "command": command
    }
    try:
        with open(json_file_path, 'r+') as file:
            data = json.load(file)
            data.append(command_data)
            file.seek(0)
            json.dump(data, file, indent=4)
    except (FileNotFoundError, json.JSONDecodeError):
        with open(json_file_path, 'w') as file:
            json.dump([command_data], file, indent=4)

def handle_received_data(data):
    """ Handle the received data from LoRa node. """
    try:
        if isinstance(data, bytes):
            data = data.decode('utf-8')  # Decoding bytes to string if necessary
        components = data.split(',')
        if len(components) == 3:
            component, component_id, command = components
            append_to_json(component, component_id, command)
        else:
            print("Received data format incorrect!")
    except Exception as e:
        print(f"Error handling received data: {str(e)}")

def clear_json_file(path):
    """ Clears a json file """
    try:
        # Open the file in write mode to clear contents
        with open(path, 'w') as file:
            file.write('[]')  
    except Exception as e:
        print(f"Failed to clear JSON file: {str(e)}")

def send_deal():
    """ Send data after taking input from the user. """
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    print("\nPlease input your commands in the format <component>,<component_id>,<command>: ", end='')
    message = input()
    DEFAULT_ADDRESS = 0
    FREQ_433 = 433
    offset_frequency = FREQ_433 - (850 if FREQ_433 > 850 else 410)
    data = (bytes([DEFAULT_ADDRESS >> 8]) + 
            bytes([DEFAULT_ADDRESS & 0xff]) +
            bytes([offset_frequency]) + 
            bytes([node.addr >> 8]) + 
            bytes([node.addr & 0xff]) + 
            bytes([node.offset_freq]) + 
            message.encode())
    node.send(data)
    print("Message sent!")
    tty.setcbreak(sys.stdin.fileno())

def main():
    """ Main function to run the script. """
    rospy.init_node("Telemetry node")
    # Clear json file
    clear_json_file(json_file_path)

    # Listen for commands to receive/send
    try:
        print("Press \033[1;32mEsc\033[0m to exit")
        print("Press \033[1;32mi\033[0m to send")
        while True:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                c = sys.stdin.read(1)
                if c == '\x1b': break
                if c == '\x69':
                    send_deal()
                sys.stdout.flush()
            
            # Received commands
            received_data = node.receive()

            # Parse received commands into json file
            if received_data:
                handle_received_data(received_data)
    except KeyboardInterrupt:
        print("\nClosing connection")
        pass
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()
