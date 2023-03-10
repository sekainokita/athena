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
twist_msg = Twist()

#2. Node - Subscriber - callback
def counter_CB(msg):
    # msg = Int32() -> -100 only called, overwrited
    print(msg.data) # msg includes a string of "data:" [value]
    twist_msg.linear.x = (msg.data % 10) / 10 #x/10
    pub.publish(twist_msg)

#3. Node - Subscriber
rospy.Subscriber('counter', Int32, callback=counter_CB)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
# do not need to set the rate value because of the rate of CB function
rospy.spin()
