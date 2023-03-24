#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from math import *
import cv2
from cv_bridge import CvBridge
import numpy as np


class Limo_camera:
    def __init__(self):
        rospy.init_node('img_compressed_node')
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback=self.camera_CB)
        
        self.cvbridge = CvBridge()

   # def img_binary(self, blend_line):
        #bin = 

    def camera_CB(self, msg):
        img = self.cvbridge.compressed_imgmsg_to_cv2(msg)
        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([0,50,0])
        upper_yellow = np.array([45,255,255])
        yellow_mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        yellow_binary = np.zeros_like(yellow_mask)
        yellow_binary[yellow_mask != 0] = 1
        print(len(yellow_mask.nonzero()[0]))

        img_out = cv2.bitwise_and(img, img, mask=yellow_mask)

        cv2.imshow("img", img)
        cv2.imshow("img_out", img_out)        
        cv2.imshow("yellow_mask", yellow_mask)
        cv2.waitKey(1)

if __name__ == "__main__":
    limo_camera = Limo_camera()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
