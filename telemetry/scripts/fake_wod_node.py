#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import String
from debra.msg import WOD, WOD_data

def generate_random_wod_data():
    return WOD_data(
        satellite_mode=bool(random.getrandbits(1)),
        battery_voltage=random.uniform(6.0, 8.0),  # Random voltage between 6.0 and 8.0 volts
        battery_current=random.uniform(0.0, 1.0),  # Random current between 0.0 and 1.0 amps
        regulated_bus_current_3v3=random.uniform(0.0, 0.5),  # Random bus current between 0.0 and 0.5 amps
        regulated_bus_current_5v=random.uniform(0.0, 0.5),  # Random bus current between 0.0 and 0.5 amps
        temperature_comm=random.uniform(15.0, 30.0),  # Random temperature between 15.0 and 30.0 Celsius
        temperature_eps=random.uniform(15.0, 30.0),  # Random temperature between 15.0 and 30.0 Celsius
        temperature_battery=random.uniform(15.0, 30.0)  # Random temperature between 15.0 and 30.0 Celsius
    )

def publish_wod():
    pub = rospy.Publisher('/wod_data', WOD, queue_size=10)
    rospy.init_node('fake_wod_node', anonymous=True)
    rate = rospy.Rate(0.2)  # 0.2 Hz

    while not rospy.is_shutdown():
        # Create the WOD message with random datasets
        wod_msg = WOD(
            satellite_id="DEBRA",
            packet_time_size=int(rospy.Time.now().to_nsec()),  # Use nanoseconds for unique unsigned int
            datasets=[generate_random_wod_data() for _ in range(32)]
        )

        pub.publish(wod_msg)
        rospy.loginfo("Publishing WOD data")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_wod()
    except rospy.ROSInterruptException:
        pass
