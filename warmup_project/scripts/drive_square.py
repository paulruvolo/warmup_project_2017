#!/usr/bin/env python

""" Uses Odometry to drive the robot in a square """

import rospy

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.init_node('drive_square')

r = rospy.Rate(10)
while not rospy.is_shutdown():
	#do the things, drive the square!