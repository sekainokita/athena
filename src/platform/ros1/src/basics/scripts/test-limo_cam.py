#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from math import *
import cv2
from cv_bridge import CvBridge

class Limo_camera:
    def __init__(self):
        rospy.init_node('img_compressed_node')
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback=self.camera_CB)
        
        self.cvbridge = CvBridge()

    def camera_CB(self, msg):
        img = self.cvbridge.compressed_imgmsg_to_cv2(msg)
        cv2.imshow("img", img)
        cv2.waitKey(1)

if __name__ == "__main__":
    limo_camera = Limo_camera()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
