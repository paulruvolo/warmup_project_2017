#!usr/bin/env python

""" avoids obstacels while driving in a desired direction """

import rospy
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

class AvoidObstacle(object):
	def __init__(self):
		rospy.init_node('obstacle_avoid')
		rospy.init_node('/stable_scan', LaserScan, self.process_scan)
		self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
		self.pub_filtered_points = rospy.Publisher('/filtered_points', Marker, queue_size= 10)
		self.obstacle_distance = 2.0
		self.scan_points = []
		self.propel_force = (10,0) #cartesian, in odom 
		self.total_force = (0,0) #polar, in base_link

	def process_scan(self, msg):
		points = []
		for angle,distance in enumerate(msg.ranges):
			if (distance < self.obstacle_distance) and (distance > 0):
				points.append(angle, distance)
		self.scan_points = points

	def cylin_to_cart(self,theta,r):
		theta_rad = math.radians(theta)
		x = r * math.cos(theta_rad)
		y = r * math.sin(theta_rad)
		return (x,y)

	def cart2pol(self, x, y):
	    rho = math.sqrt(x**2 + y**2)
	    phi = math.arctan2(y, x)
	    return(rho, phi)

	def sum_force(self):
		total_force = self.propel_force
		for point in self.scan_points:
			point_force = self.cylin_to_cart(angle,-1/distance)
			total_force[0] += point_force[0] #sum the x and y components of the force
			total_force[1] += point_force[1]
		self.total_force = total_force




