#!/usr/bin/env python

import rospy
from datetime import datetime

def log_time():
    rospy.init_node('time_logger_node', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    with open('time_log.txt', 'a') as log_file:
        while not rospy.is_shutdown():
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            log_file.write(f"{current_time}\n")
            rospy.loginfo(f"Logged time: {current_time}")
            rate.sleep()

if __name__ == '__main__':
    try:
        log_time()
    except rospy.ROSInterruptException:
        pass
