#!/usr/bin/env python
import rospy


####################################################################################################################
'''                                                                                                               #
This program is used for logging data from rosbag into a CSV format for further data analysis                     #
                                                                                                                  #
It works by subscribing to all of the messages and serializing the message data into a string                     #
                                                                                                                  #
One of the subscribers is a "trigger". In other words, when a message is received on this specific topic,         #
all of the most recent messages from all the topics are logged as a row in the CSV document                       #
                                                                                                                  #
Therefore, you should select the topic that is publishing at the slowest frequency as the trigger so that         #
a message isn't logged twice    

Limitation is that you can only have one trigger node                                                             #
'''                                                                                                               #
###################################################################################################################


rospy.init_node("data_logger")

# Instantiate the trigger subscriber
trigger_sub = rospy.Subscriber("")

# Create a dictionary containing the other subscribers
other_subs = {}