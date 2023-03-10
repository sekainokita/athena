#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

from math import * # * means All

#1. Node - Init Setting
rospy.init_node('pub_node')

#2. Node - Publisher or Subscriber
pub = rospy.Publisher('counter', Int32, queue_size=1)

#3. Rate - Setting
rate = rospy.Rate(10) # Send a data 10 times in 1s

counter = 0
while   not rospy.is_shutdown():
#4. Node - publisher - publish
    print("counter:",counter)
    pub.publish(counter)
    counter +=   1
    #5. Rating
    rate.sleep() # Not sleep as C/C++, repeat as the rate value