#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *
from time import *

class Limo_lidar:
    def __init__(self):
        rospy.init_node('laser_scan_node')
        rospy.Subscriber("/scan", LaserScan, callback=self.laser_CB)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
        self.lidar_flag = False
        self.deg = 10
        self.cmd_vel_msg = Twist()
        self.first_time = time()

    def laser_CB(self, msg):
        self.second_time = time()
        num = 0
        # print(f"angle_min : {msg.angle_min*180/pi}")
        # print(f"angle_max : {msg.angle_max*180/pi}")
        # print(f"angle_increment : {msg.angle_increment*180/pi}")
        # print(f"len_ranges : {len(msg.ranges)}")
        
        if self.lidar_flag == False:
            self.degrees = [(msg.angle_min+(i*msg.angle_increment))*180/pi for i, data in enumerate(msg.ranges)]
            self.lidar_flag = True

        for i, data in enumerate(msg.ranges):
            if -self.deg < self.degrees[i] < self.deg and 0 < msg.ranges[i] < 0.5:
                #print(f"{self.degrees[i]} : {msg.ranges[i]}")
                # 50cm
                num+=1

        if num < 10:
            self.cmd_vel_msg.linear.x = 1
            print("START")
        else:
            self.cmd_vel_msg.linear.x = 0
            print("STOP")

        if self.second_time - self.first_time > 0.5:
            self.pub.publish(self.cmd_vel_msg)
#        print(f'degrees:{degrees}')

if __name__ == "__main__":
    limo_lidar = Limo_lidar()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
