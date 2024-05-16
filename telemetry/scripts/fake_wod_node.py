#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from debra.msg import WOD, WOD_data

def publish_wod():
    pub = rospy.Publisher('/wod_data', WOD, queue_size=10)
    rospy.init_node('fake_wod_node', anonymous=True)
    rate = rospy.Rate(0.2)  # 0.2 Hz

    # Define the datasets with float values
    dataset1 = WOD_data(
        satellite_mode=True,
        battery_voltage=6.5,  # Example voltage in volts
        battery_current=0.5,  # Example current in amps
        regulated_bus_current_3v3=0.1,  # Example bus current in amps
        regulated_bus_current_5v=0.2,  # Example bus current in amps
        temperature_comm=25.0,  # Example temperature in Celsius
        temperature_eps=20.0,  # Example temperature in Celsius
        temperature_battery=15.0  # Example temperature in Celsius
    )
    
    dataset2 = WOD_data(
        satellite_mode=False,
        battery_voltage=7.0,  # Example voltage in volts
        battery_current=0.6,  # Example current in amps
        regulated_bus_current_3v3=0.2,  # Example bus current in amps
        regulated_bus_current_5v=0.3,  # Example bus current in amps
        temperature_comm=26.0,  # Example temperature in Celsius
        temperature_eps=21.0,  # Example temperature in Celsius
        temperature_battery=16.0  # Example temperature in Celsius
    )

    # Create the WOD message
    wod_msg = WOD(
        satellite_id="DEBRA",
        packet_time_size=1234567890,
        datasets=[dataset1, dataset2] + [WOD_data() for _ in range(30)] 
    )

    while not rospy.is_shutdown():
        pub.publish(wod_msg)
        rospy.loginfo("Publishing WOD data")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_wod()
    except rospy.ROSInterruptException:
        pass
