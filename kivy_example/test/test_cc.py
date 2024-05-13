#!/usr/bin/env python

import rospy
from hb_common_msgs.msg import CleaningFeedback, CleaningFeedbackArray


class Feedback:
    def __init__(self):
        self._publisher = rospy.Publisher(
            "/cleaning_cores/feedback", CleaningFeedbackArray, queue_size=1
        )

        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / 3), self.pub_commands)

    def pub_commands(self, *args):
        """Publish command and feedback frames"""

        cleaning_feedback_array = CleaningFeedbackArray()
        cleaning_feedback_array.header.stamp = rospy.get_rostime()
        cleaning_feedback = CleaningFeedback()
        cleaning_feedback.id = 1
        cleaning_feedback.feedback.rpm = 100
        cleaning_feedback_array.status.append(cleaning_feedback)

        cleaning_feedback = CleaningFeedback()
        cleaning_feedback.id = 2
        cleaning_feedback.feedback.rpm = 653
        cleaning_feedback_array.status.append(cleaning_feedback)

        cleaning_feedback = CleaningFeedback()
        cleaning_feedback.id = 3
        cleaning_feedback.feedback.rpm = 0
        cleaning_feedback.feedback.errors = [4, 0, 0, 3, 0]

        cleaning_feedback_array.status.append(cleaning_feedback)

        cleaning_feedback = CleaningFeedback()
        cleaning_feedback.id = 4
        cleaning_feedback.feedback.rpm = 23
        cleaning_feedback.feedback.errors = [4, 0, 2, 0, 0]
        cleaning_feedback_array.status.append(cleaning_feedback)

        self._publisher.publish(cleaning_feedback_array)


if __name__ == "__main__":
    rospy.init_node("test_ccs")
    rospy.loginfo("Test node")
    cf = Feedback()
    rospy.spin()
