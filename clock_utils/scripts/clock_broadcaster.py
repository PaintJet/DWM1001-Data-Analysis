#!/usr/bin/env python


# import required packages
import rospy
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock

# Callback to receive tf messages
def tf_callback(message):
    for transform in message.transforms:
        frame_id = transform.header.frame_id
        child_id = transform.child_frame_id

        # Check if this message is the correct transform
        if(frame_id == "odom" and child_id == "base_footprint"):
            #print("{} {}".format(transform.header.stamp.secs, transform.header.stamp.nsecs))

            clock_msg = Clock()
            clock_msg.clock.secs = transform.header.stamp.secs
            clock_msg.clock.nsecs = transform.header.stamp.nsecs
            clock_pub.publish(clock_msg)
            break

# Initialize rospy node
rospy.init_node("Clock_publisher")

# Initialize subscribers
tf_sub = rospy.Subscriber("/tf", TFMessage, tf_callback)

# Publisher to publish time on the clock topic
clock_pub = rospy.Publisher("/clock", Clock, queue_size = 0)

rospy.spin()








