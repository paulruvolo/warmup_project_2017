#!/usr/bin/env python

""" Determines the closest large object in front of the robot and follows it. """

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FollowPerson(object):
	def __init__(self):
		rospy.init_node('person_follow')
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	def process_scan(self):
		pass

if __name__ == '__main__':
	node = FollowPerson()
	print("Node closed.")


