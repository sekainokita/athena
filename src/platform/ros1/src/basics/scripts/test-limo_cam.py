#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from math import *
import cv2
from cv_bridge import CvBridge
import numpy as np
from time import *

class Limo_camera:
    def __init__(self):
        rospy.init_node('img_compressed_node')
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback=self.camera_CB)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)        
        self.cmd_vel_msg = Twist()
        self.cvbridge = CvBridge()
        self.first_time = time()

    def camera_CB(self, msg):
        self.second_time = time()
        num = 0        
        img = self.cvbridge.compressed_imgmsg_to_cv2(msg)
        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([0,50,0])
        upper_yellow = np.array([45,255,255])
        yellow_mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        yellow_binary = np.zeros_like(yellow_mask)
        yellow_binary[yellow_mask != 0] = 1
        print(len(yellow_mask.nonzero()[0]))

        img_out = cv2.bitwise_and(img, img, mask=yellow_mask)

        if len(yellow_mask.nonzero()[0]) < 5000:
            self.cmd_vel_msg.linear.x = 1 # speed
            print(f"START")
        else:
            self.cmd_vel_msg.linear.x = 0
            print(f"STOP")

        if self.second_time - self.first_time > 1000:
            self.first_time = self.second_time

        self.pub.publish(self.cmd_vel_msg)

        #cv2.imshow("img", img)
        #cv2.imshow("img_out", img_out)        
        #cv2.imshow("yellow_mask", yellow_mask)
        #cv2.waitKey(1)

if __name__ == "__main__":
    limo_camera = Limo_camera()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
