#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped
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


def publish_ground_truth(estimated_msg):
    global seq
    ground_truth = rospy.get_param('/ground_truth')

    pose_msg = PoseStamped()

    # Write position to message
    pose_msg.pose.position.x = ground_truth[0]
    pose_msg.pose.position.y = ground_truth[1]
    pose_msg.pose.position.z = 0

    # Write orientation to message
    pose_msg.pose.orientation.x = 0
    pose_msg.pose.orientation.y = 0
    pose_msg.pose.orientation.z = np.sin(ground_truth[2]/2)
    pose_msg.pose.orientation.w = np.cos(ground_truth[2]/2)

    # Write header
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"
    pose_msg.header.seq = seq

    seq += 1

    ground_truth_pub.publish(pose_msg)

    # Write to file
    (_, _,  yaw_estimated) = tf_conversions.transformations.euler_from_quaternion([estimated_msg.pose.orientation.x, estimated_msg.pose.orientation.y, estimated_msg.pose.orientation.z, estimated_msg.pose.orientation.w])

    f.write("{},{},{},{},{},{}\n".format(ground_truth[0], ground_truth[1], ground_truth[2], estimated_msg.pose.position.x, estimated_msg.pose.position.y, yaw_estimated))

# Sequence for pose messages
seq = 0

# Init the ros node
rospy.init_node("ground_truth_publisher")

rospy.Subscriber("/uwb/pf/pose", PoseStamped, publish_ground_truth)
ground_truth_pub = rospy.Publisher('/ground_truth', PoseStamped, queue_size = 10)

# Open logging file
f = open("estimated_vs_ground_truth_pose.csv", "w+")
f.write("Ground truth x, Ground truth y, Ground truth theta, Estimated x, Estimated y, Estimated theta\n")


rospy.loginfo("Successfully initiliazed ground truth publisher")


while not rospy.is_shutdown():
    # Stay in the loop
    rospy.spin()

# Close logging file
f.close()