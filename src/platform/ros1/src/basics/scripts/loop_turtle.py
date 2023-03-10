#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import * # * means All

#1. Node - Init Setting
rospy.init_node('sub_node')

# Example of a, b
#a = Int32()
#a.data
#b = Twist()
#b.angular.x
twist_msg = Twist()

pose_msg = Pose()

#2. Node - Subscriber - callback
def counter_CB(msg):
    print(msg.x)
    twist_msg.linear.x = 10 - msg.x
    pub.publish(twist_msg)

#3. Node - Subscriber
rospy.Subscriber('/turtle1/pose', Pose, callback=counter_CB)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
# do not need to set the rate value because of the rate of CB function
rospy.spin()
