#!/usr/bin/env python
# license removed for brevity
import rospy
from hb_drivers_msgs.msg import TimeOfFlight, TimeOfFlightArray
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher("/drivers/tofs/data", TimeOfFlightArray, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    val = 1.2
    while not rospy.is_shutdown():
        msg = TimeOfFlightArray()

        # print(msg.ranges)
        for i in range(8):
            el = TimeOfFlight()
            if val > 1:
                el.range = float("inf")
            else:
                el.range = val
            el.status = 0
            msg.ranges.append(el)
        val = val - 0.01
        if val < 0:
            val = 1.2
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub.publish(msg)
        # break
        # print(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
