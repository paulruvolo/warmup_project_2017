#!/usr/bin/env python

from neato_node.msg import Bump
from geometry_msgs.msg import Twist

import rospy

def sub_callback(msg):
	pass

rospy.init_node('emergency_stop')
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
rospy.Subscriber('/bump', Bump, sub_callback)

r = rospy.Rate(20)
while not rospy.is_shutdown():
	r.sleep()