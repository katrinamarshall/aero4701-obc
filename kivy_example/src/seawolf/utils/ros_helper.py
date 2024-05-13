import rosgraph
import rospy
import rostopic


def get_topics_by_type(*args):
    all_topics = rostopic.get_topic_list()
    requested_topics = []

    for msg_type in args:
        # type is the actual msg name as reported in the ROS graph
        # eg sensor.msgs.CompressedImage type is sensor_msgs/CompressedImage
        msg_type_name = msg_type._type

        for t in all_topics:
            for topic, msg_name, _ in t:
                # if the topic has the requested type
                if msg_name == msg_type_name:
                    # add a tuple of topic and the message type
                    requested_topics.append((topic, msg_type))

    if len(requested_topics) == 0:
        return None
    else:
        return requested_topics


def get_compressed_image_topics():
    print(rostopic.get_topic_list())
