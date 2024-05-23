#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from eps.msg import current_voltage
from thermal.msg import temperatures  # Ensure correct import path
    
class SSHControl:
    def __init__(self):

        # Publishers
        self.pub_ssh_command = rospy.Publisher('/state_ssh_command', Int8, queue_size=10)
        self.pub_temp_command = rospy.Publisher('/temperatures', temperatures, queue_size=10)
        self.pub_battery_command = rospy.Publisher('/current_voltage', current_voltage, queue_size=10)

    def get_command(self):
        user_input = input("Enter command (state 1-8, 't XX' for temperature, or 'v XX' for voltage): ")

        if user_input.startswith('t '):
            try:
                temp_msg = temperatures()
                temp_msg.thermistor_1 = float(user_input.split()[1])
                self.pub_temp_command.publish(temp_msg)
                rospy.loginfo(f"Temperature set to {temp_msg}")
            except (IndexError, ValueError):
                rospy.logwarn("Invalid temperature input, please enter 't XX' where XX is a number.")
        elif user_input.startswith('v '):
            try:
                v_msg = current_voltage()
                v_msg.voltage_40 = float(user_input.split()[1])
                self.pub_battery_command.publish(v_msg)
                rospy.loginfo(f"Battery voltage set to {v_msg}")
            except (IndexError, ValueError):
                rospy.logwarn("Invalid voltage input, please enter 'v XX' where XX is a number.")
        else:
            try:
                new_state = int(user_input)
                if 1 <= new_state <= 8:
                    self.pub_ssh_command.publish(Int8(new_state))
                    rospy.loginfo(f"State set to {new_state}")
                else:
                    rospy.logwarn("Input out of range, please enter a number between 1 and 8.")
            except ValueError:
                rospy.logwarn("Invalid input, please enter a valid command.")

if __name__ == '__main__':
    rospy.init_node('debra_ssh_node')
    debra_ssh = SSHControl()
    while True:
        debra_ssh.get_command()
    rospy.spin()
