#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print(msg.ranges[360])

rospy.init_node('scan_values')
sub = rospy.Subscriber('/macroed/laser/scan', LaserScan, callback)
rospy.spin()