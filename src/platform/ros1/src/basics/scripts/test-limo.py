#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node('laser_scan_node')
laser_data = LaserScan()

def laser_CB(msg):
    print(msg)

rospy.Subscriber("/scan", LaserScan, callback=laser_CB)

rospy.spin()
