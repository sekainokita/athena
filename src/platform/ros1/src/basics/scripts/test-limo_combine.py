#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from math import *
import cv2
from cv_bridge import CvBridge
import numpy as np

class Limo_Combine:
    def __init__(self):
        rospy.init_node('combine_node')

        # image
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback=self.camera_CB)
        
        # Lidar
        rospy.Subscriber("/scan", LaserScan, callback=self.laser_CB)
        self.deg = 10

        # ros
        self.pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=3)
        self.pub_img = rospy.Publisher("image/compressed", CompressedImage, queue_size=10)
        self.cmd_vel_msg = Twist()
        self.cvbridge = CvBridge()
        self.first_time = rospy.get_time()
        self.lidar_flag = False
        self.lidar_stop_flag = False
        self.camera_stop_flag = False
        self.rate = rospy.Rate(10)

    def laser_CB(self, msg):
        num = 0
        if self.lidar_flag == False:
            self.degrees = [(msg.angle_min+(i*msg.angle_increment))*180/pi for i, data in enumerate(msg.ranges)]
            self.lidar_flag = True

        for i, data in enumerate(msg.ranges):
            if -self.deg < self.degrees[i] < self.deg and 0 < msg.ranges[i] < 0.5:
                num+=1

        if num < 10:
            self.lidar_stop_flag = False
        else:
            self.lidar_stop_flag = True

    def camera_CB(self, msg):
        num = 0        
        img = self.cvbridge.compressed_imgmsg_to_cv2(msg)
        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([0,50,0])
        upper_yellow = np.array([45,255,255])
        yellow_mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
        yellow_binary = np.zeros_like(yellow_mask)
        yellow_binary[yellow_mask != 0] = 1
        img_out = cv2.bitwise_and(img, img, mask=yellow_mask)

        if len(yellow_mask.nonzero()[0]) < 5000:
            self.camera_stop_flag = False
        else:
            self.camera_stop_flag = True

        img_msg = self.cvbridge.cv2_to_compressed_imgmsg(img_out)
        self.pub_img.publish(img_msg)
        #cv2.imshow("img_out", img_out)        
        #cv2.waitKey(1)

    def main(self):
        self.second_time = rospy.get_time()
        if self.second_time - self.first_time > 0.07: # 0.1s
            if (self.lidar_stop_flag == True) or (self.camera_stop_flag == True):
                self.cmd_vel_msg.linear.x = 0
                print(f"STOP")
            else:
                self.cmd_vel_msg.linear.x = 0.5
                print(f"START")

            print(f"lidar{self.lidar_stop_flag}, camera{self.camera_stop_flag}")

            self.pub_vel.publish(self.cmd_vel_msg)
            self.rate.sleep()

if __name__ == "__main__":
    limo_combine = Limo_Combine()
    try:
        while not rospy.is_shutdown():
            limo_combine.main()
    except rospy.ROSInterruptException:
        pass
