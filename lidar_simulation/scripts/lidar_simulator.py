#!/usr/bin/env python

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Pose

rospy.init_node("lidar_simulator")

# Create occupancy grid
grid = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100],
    [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]], dtype = np.int8)
grid = grid.flatten()

ranges = np.zeros(shape = (90,), dtype = np.float32)

for i in range(ranges.shape[0]):
    # Break measurements into 90 segments, one scan every 1 degree
    angle = i*(np.pi/2/90)

    if(angle <= np.pi/4):
        ranges[i] = 1/np.cos(angle)
    else:
        ranges[i] = 1/np.cos(np.pi/2 - angle)



map_load_time = rospy.Time.now()

map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)


rate = rospy.Rate(10)

seq = 0
while not rospy.is_shutdown():

    map_msg = OccupancyGrid()

    map_msg.data = grid

    # header
    map_msg.header.seq = seq
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = "map"

    # info
    map_msg.info.map_load_time = map_load_time
    map_msg.info.resolution = 0.1 # meters
    map_msg.info.width = 11
    map_msg.info.height = 11

    map_pose = Pose()
    map_pose.position.x = 0
    map_pose.position.y = 0
    map_pose.position.z = 0

    map_pose.orientation.x = 0
    map_pose.orientation.y = 0
    map_pose.orientation.z = 0
    map_pose.orientation.w = 1

    map_msg.info.origin = map_pose

    map_pub.publish(map_msg)

    scan = LaserScan()

    # header
    scan.header.seq = seq
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = "laser"

    scan.angle_min = 3*np.pi/3
    scan.angle_max = 2*np.pi
    scan.angle_increment = (np.pi/2/90)
    scan.range_min = 0.0
    scan.range_max = 10.0
    scan.ranges = ranges

    scan_pub.publish(scan)

    seq += 1

    rate.sleep()





