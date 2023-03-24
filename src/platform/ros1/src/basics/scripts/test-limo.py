#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *

class Limo_lidar:
    def __init__(self):
        rospy.init_node('laser_scan_node')
        rospy.Subscriber("/scan", LaserScan, callback=self.laser_CB)
        self.lidar_flag = False

    def laser_CB(self, msg):
        # print(f"angle_min : {msg.angle_min*180/pi}")
        # print(f"angle_max : {msg.angle_max*180/pi}")
        # print(f"angle_increment : {msg.angle_increment*180/pi}")
        # print(f"len_ranges : {len(msg.ranges)}")
        
        if self.lidar_flag == False:
            self.degrees = [(msg.angle_min+(i*msg.angle_increment))*180/pi for i, data in enumerate(msg.ranges)]
            self.lidar_flag = True
        
        for i, data in enumerate(msg.ranges):
            if -30 < self.degrees[i] < 30:
                print(f"{self.degrees[i]} : {msg.ranges[i]}")
                pass

#        print(f'degrees:{degrees}')

if __name__ == "__main__":
    limo_lidar = Limo_lidar()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
