#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
import tf_conversions # Use to convert from Euler angles to quaternions
import numpy as np


####################################################################################################################
'''                                                                                                               #
This program is used for publishing ground truth data of the state of the robot for later analysis                #
In addition to publishing the ground truth, the program also logs the ground truth and estimated state            #
into a csv file.                                                                                                  #
                                                                                                                  #
Every time an estimated pose message is publishes, this node reads the rosparam /ground_truth and publishes       #
the values, as well as writes the ground truth and estimated position to the csv file                             #
'''                                                                                                               #
###################################################################################################################


def republish_ground_truth(msg_str):
    global seq

    # Split pose into array
    try:
        # Split string into individual elements and convert to floats
        pose = [float(x) for x in msg_str.data.split()]

        print(pose)

        pose_msg = PoseStamped()

        # Write position to message
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = 0

        # Write orientation to message
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = np.sin(pose[2]/2)
        pose_msg.pose.orientation.w = np.cos(pose[2]/2)

        # Write header
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.header.seq = seq

        seq += 1
        ground_truth_pub.publish(pose_msg)
    except:
        return



# Sequence for pose messages
seq = 0

# Init the ros node
rospy.init_node("ground_truth_republisher")

rospy.Subscriber("/ground_truth", String, republish_ground_truth)
ground_truth_pub = rospy.Publisher('/ground_truth_pose', PoseStamped, queue_size = 10)


rospy.loginfo("Successfully initiliazed ground truth publisher")


while not rospy.is_shutdown():
    # Stay in the loop
    rospy.spin()

