#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *

rospy.init_node('laser_scan_node')
laser_data = LaserScan()

def laser_CB(msg):
    print(f"angle_min : {msg.angle_min*180/pi}")
    print(f"angle_max : {msg.angle_max*180/pi}")
    print(f"angle_increment : {msg.angle_increment*180/pi}")
    print(f"len_ranges : {len(msg.ranges)}")

    degrees = [
                (msg.angle_min+(i*msg.angle_increment))*180/pi
                for i, data in enumerate(msg.ranges)
            ]
    
    print(f'degrees:{degrees}')


rospy.Subscriber("/scan", LaserScan, callback=laser_CB)

rospy.spin()
