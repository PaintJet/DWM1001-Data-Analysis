import rospy
from std_msgs.msg import String, Float64

import numpy as np

import Queue


rospy.init_node("range_accuracy_logger")

ground_truth = []
ground_truth_time = 0.0


writeQ = Queue.Queue()


anchor_positions = np.array([
                                       [0.0, 0.0],
                                       [18.28, -2.28],
                                       [18.28, 7.01],
                                       [0.00, 4.62],
                                       [7.92, -1.52],
                                       [0.0, 0.0],
                                       [0.0, 0.0],
                                       [7.92, 5.72],
                                       [0.0, 0.0]
                                    ])
tag_transforms = np.array([
                                       [0, 0.145],
                                       [0.145, 0],
                                       [-0.145, 0]
                                    ])


# create_uwb_observation_function
# Factory function that generates an observation function for each of the UWB anchors
# The observation function will craft the observation input into the particle filter and feed into the pf
# tag_number - Which tag on the robot is it
# anchor_number - Which anchor in the environment is it
# num_tags - Total number of UWB tags on the robot
# num_anchors - Total number of anchors in the environment
def create_uwb_observation_function(tag_number, anchor_number):
    def update_pf(message):
        # Get global arrays for sensor measurements
        global ground_truth, ground_truth_time, tag_transforms, anchor_positions, writeQ

        if(rospy.Time.now().secs - ground_truth_time < 1):
            # Position of the tag relative to the map
            x_tag = tag_transforms[tag_number][0] * np.cos(ground_truth[2]) - tag_transforms[tag_number][1] * np.sin(ground_truth[2]) + ground_truth[0]
            y_tag = tag_transforms[tag_number][0] * np.sin(ground_truth[2]) + tag_transforms[tag_number][1] * np.cos(ground_truth[2]) + ground_truth[1]
            tag_position = np.array([x_tag, y_tag])

            ground_truth_observation = np.linalg.norm(tag_position - anchor_positions[anchor_number])

            writeQ.put("{},{},{},{}\n".format(tag_number, anchor_number, message.data, ground_truth_observation))

    return update_pf

def log_ground_truth(msg):
    global ground_truth, ground_truth_time

    ground_truth_arr = msg.data.split(" ")
    ground_truth = [float(x) for x in ground_truth_arr]

    ground_truth_time = rospy.Time.now().secs


anchor_distance_subs = [
        rospy.Subscriber("/uwb/0/anchors/9205/distance", Float64, create_uwb_observation_function(0, 0)),
        rospy.Subscriber("/uwb/0/anchors/C518/distance", Float64, create_uwb_observation_function(0, 1)),
        rospy.Subscriber("/uwb/0/anchors/9AAB/distance", Float64, create_uwb_observation_function(0, 2 )),
        rospy.Subscriber("/uwb/0/anchors/D81B/distance", Float64, create_uwb_observation_function(0, 3 )),
        rospy.Subscriber("/uwb/0/anchors/998D/distance", Float64, create_uwb_observation_function(0, 4 )),
        rospy.Subscriber("/uwb/0/anchors/D499/distance", Float64, create_uwb_observation_function(0, 5 )),
        rospy.Subscriber("/uwb/0/anchors/9BAC/distance", Float64, create_uwb_observation_function(0, 6 )),
        rospy.Subscriber("/uwb/0/anchors/1C31/distance", Float64, create_uwb_observation_function(0, 7 )),
        rospy.Subscriber("/uwb/0/anchors/91BA/distance", Float64, create_uwb_observation_function(0, 8 )),

        rospy.Subscriber("/uwb/1/anchors/9205/distance", Float64, create_uwb_observation_function(1, 0 )),
        rospy.Subscriber("/uwb/1/anchors/C518/distance", Float64, create_uwb_observation_function(1, 1 )),
        rospy.Subscriber("/uwb/1/anchors/9AAB/distance", Float64, create_uwb_observation_function(1, 2 )),
        rospy.Subscriber("/uwb/1/anchors/D81B/distance", Float64, create_uwb_observation_function(1, 3 )),
        rospy.Subscriber("/uwb/1/anchors/998D/distance", Float64, create_uwb_observation_function(1, 4 )),
        rospy.Subscriber("/uwb/1/anchors/D499/distance", Float64, create_uwb_observation_function(1, 5 )),
        rospy.Subscriber("/uwb/1/anchors/9BAC/distance", Float64, create_uwb_observation_function(1, 6 )),
        rospy.Subscriber("/uwb/1/anchors/1C31/distance", Float64, create_uwb_observation_function(1, 7 )),
        rospy.Subscriber("/uwb/1/anchors/91BA/distance", Float64, create_uwb_observation_function(1, 8 )),

        rospy.Subscriber("/uwb/2/anchors/9205/distance", Float64, create_uwb_observation_function(2, 0 )),
        rospy.Subscriber("/uwb/2/anchors/9AAB/distance", Float64, create_uwb_observation_function(2, 1 )),
        rospy.Subscriber("/uwb/2/anchors/C518/distance", Float64, create_uwb_observation_function(2, 2 )),
        rospy.Subscriber("/uwb/2/anchors/D81B/distance", Float64, create_uwb_observation_function(2, 3 )),
        rospy.Subscriber("/uwb/2/anchors/998D/distance", Float64, create_uwb_observation_function(2, 4 )),
        rospy.Subscriber("/uwb/2/anchors/D499/distance", Float64, create_uwb_observation_function(2, 5 )),
        rospy.Subscriber("/uwb/2/anchors/9BAC/distance", Float64, create_uwb_observation_function(2, 6 )),
        rospy.Subscriber("/uwb/2/anchors/1C31/distance", Float64, create_uwb_observation_function(2, 7 )),
        rospy.Subscriber("/uwb/2/anchors/91BA/distance", Float64, create_uwb_observation_function(2, 8 ))
    ]


rospy.Subscriber("/ground_truth", String, log_ground_truth)

f = open("uwb_range_accuracy.csv", "w+")
f.write("Tag:,tAnchor:,Observed:,Expected:\n")

while not rospy.is_shutdown():
    if(not writeQ.empty()):
        f.write(writeQ.get())

f.close()