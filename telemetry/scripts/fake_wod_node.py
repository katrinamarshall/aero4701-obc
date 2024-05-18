#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import String
from debra.msg import WOD, WOD_data, payload_data, satellite_pose

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

def generate_random_payload_data():
    return payload_data(
        debris_position_x=random.uniform(-100.0, 100.0),
        debris_position_y=random.uniform(-100.0, 100.0),
        debris_position_z=random.uniform(-100.0, 100.0),
        debris_velocity_x=random.uniform(-10.0, 10.0),
        debris_velocity_y=random.uniform(-10.0, 10.0),
        debris_velocity_z=random.uniform(-10.0, 10.0),
        debris_diameter=random.uniform(0.1, 10.0),
        time_of_detection=random.randint(0, 100000),
        object_count=random.randint(0, 100)
    )

def generate_random_satellite_pose():
    return satellite_pose(
        position_x=random.uniform(-1000.0, 1000.0),
        position_y=random.uniform(-1000.0, 1000.0),
        position_z=random.uniform(-1000.0, 1000.0),
        orientation_x=random.uniform(-1.0, 1.0),
        orientation_y=random.uniform(-1.0, 1.0),
        orientation_z=random.uniform(-1.0, 1.0),
        orientation_w=random.uniform(-1.0, 1.0),
        velocity_x=random.uniform(-100.0, 100.0),
        velocity_y=random.uniform(-100.0, 100.0),
        velocity_z=random.uniform(-100.0, 100.0)
    )

def publish_fake_data():
    pub_wod = rospy.Publisher('/wod_data', WOD, queue_size=10)
    pub_payload = rospy.Publisher('/payload_data', payload_data, queue_size=10)
    pub_pose = rospy.Publisher('/satellite_pose_data', satellite_pose, queue_size=10)
    
    rospy.init_node('fake_data_node', anonymous=True)
    rate = rospy.Rate(0.2)  # 0.2 Hz

    while not rospy.is_shutdown():
        # Create and publish WOD message with random datasets
        wod_msg = WOD(
            satellite_id="DEBRA",
            packet_time_size=int(rospy.Time.now().to_sec()),  # Use seconds for unsigned int
            datasets=[generate_random_wod_data() for _ in range(32)]
        )
        pub_wod.publish(wod_msg)
        rospy.loginfo("Publishing WOD data")

        # Create and publish random payload data
        payload_msg = generate_random_payload_data()
        pub_payload.publish(payload_msg)
        rospy.loginfo("Publishing Payload data")

        # Create and publish random satellite pose data
        pose_msg = generate_random_satellite_pose()
        pub_pose.publish(pose_msg)
        rospy.loginfo("Publishing Satellite Pose data")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_data()
    except rospy.ROSInterruptException:
        pass
