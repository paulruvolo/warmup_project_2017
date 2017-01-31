#!/usr/bin/env python

""" Determines the closest large object in front of the robot and follows it. """

import rospy
import math
import numpy as np
from sklearn.cluster import KMeans
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FollowPerson(object):
	def __init__(self):
		rospy.init_node('person_follow')
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.scan_points = None
		self.scan_width = 22

	def process_scan(self, msg):
		points = np.array([])
		
		for i in range(0, self.scan_width):
			pass
			# msg.ranges[i]
		for i in range(359-self.scan_width, 359):
			pass
			# msg.ranges[i]

if __name__ == '__main__':
	node = FollowPerson()
	print("Node closed.")

	# identify closest large object