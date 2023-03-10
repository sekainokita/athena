#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from math import * # * means All

#1. Node - Init Setting
rospy.init_node('sub_node')

# Example of a, b
#a = Int32()
#a.data
#b = Twist()
#b.angular.x

#2. Node - Subscriber - callback
def counter_CB(msg):
    # msg = Int32() -> -100 only called, overwrited
    print(msg.data - 100) # msg includes a string of "data:" [value]
    pass

#3. Node - Subscriber
rospy.Subscriber('counter', Int32, callback=counter_CB)
rospy.spin()
