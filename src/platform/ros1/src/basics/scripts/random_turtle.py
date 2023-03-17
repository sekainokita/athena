#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from random import *

#1. Node - Init Setting
rospy.init_node('pub_sub_node')

twist_msg = Twist()
pose_msg = Pose()

#2. Node - Subscriber - callback
def counter_CB(msg):
    print(msg.x)
    if 1<= msg.x <= 10 and 1 <= msg.y <=10 :
        twist_msg.linear.x(random(0,1)*2)
        twist_msg.angular.z((random()*4-2))
    else :
        twist_msg.linear.x = 0.3
        twist_msg.angular.x = 2

    pub.publish(twist_msg)

#3. Node - Subscriber
rospy.Subscriber('turtle1/pose', Pose, callback=counter_CB)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
rospy.spin()
