#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from debra.msg import WOD, WOD_data

def publish_wod():
    pub = rospy.Publisher('/wod_data', WOD, queue_size=10)
    rospy.init_node('fake_wod_node', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz

    # Define the datasets
    dataset1 = WOD_data(
        satellite_mode=1,
        battery_voltage=128,
        battery_current=64,
        regulated_bus_current_3v3=32,
        regulated_bus_current_5v=16,
        temperature_comm=8,
        temperature_eps=4,
        temperature_battery=2
    )
    
    dataset2 = WOD_data(
        satellite_mode=2,
        battery_voltage=129,
        battery_current=65,
        regulated_bus_current_3v3=33,
        regulated_bus_current_5v=17,
        temperature_comm=9,
        temperature_eps=5,
        temperature_battery=3
    )

    # Create the WOD message
    wod_msg = WOD(
        satellite_id="DEBRA",
        packet_time_size=1234567890,
        datasets=[dataset1, dataset2] + [WOD_data() for _ in range(30)] # Pad with 30 empty datasets
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
