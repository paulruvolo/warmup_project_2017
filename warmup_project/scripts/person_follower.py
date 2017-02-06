#!/usr/bin/env python

""" Determines the closest large object in front of the robot and follows it. """

import rospy
import math
import numpy as np
from sklearn.cluster import KMeans
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

class FollowPerson(object):
	def __init__(self):
		rospy.init_node('person_follow')
		rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
		self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
		self.pub_filtered_points = rospy.Publisher('/filtered_points', Marker, queue_size= 10)
		self.pub_cluster = rospy.Publisher('/cluster', Marker, queue_size= 10)
		self.scan_width = 30
		self.scan_points = np.zeros((1,2))
		self.too_far = 1.0 #meters, ignore objects beyond this distance
		self.kmeans = None
		self.person_present = False
		self.closest_cluster = None
		self.twist = Twist()

		self.angle_k = 1.5
		self.distance_k = 0.7
		self.goal_distance = 0.6
		self.goal_point = Point()
		self.distance_error = None
		self.angle_error = None

		self.num_clusters = 5

	def process_scan(self, msg):
		points = np.zeros((self.scan_width*2,2))
		for i in range(0, self.scan_width):
			if (msg.ranges[i] < self.too_far) and (msg.ranges[i] > 0.0):
				points[i] = self.cylin_to_cart(i,msg.ranges[i])
		for i in range(360-self.scan_width, 360):
			if (msg.ranges[i] < self.too_far) and (msg.ranges[i] > 0.0):
				points[i-(360-2*self.scan_width)] = self.cylin_to_cart(i,msg.ranges[i])

		self.scan_points = points[~np.all(points == 0, axis = 1)] #set only the non-zero points

	def cylin_to_cart(self,theta,r):
		theta_rad = math.radians(theta)
		x = r * math.cos(theta_rad)
		y = r * math.sin(theta_rad)
		return [x,y]

	def compute_kmeans(self):
		if len(self.scan_points) >= self.num_clusters:
			self.kmeans = KMeans(n_clusters=self.num_clusters, random_state=0).fit(self.scan_points)
			self.person_present = True
		else:
			self.kmeans = None
			self.person_present = False

	def show_cluster(self):
		x = self.closest_cluster[0][0]
		y = self.closest_cluster[0][1]

		marker_msg = Marker()
		marker_msg.type = marker_msg.SPHERE
		marker_msg.action = marker_msg.ADD
		marker_msg.pose.position.x = x
		marker_msg.pose.position.y = y
		marker_msg.pose.position.z = 0.4
		marker_msg.scale.x = 0.2
		marker_msg.scale.y = 0.2
		marker_msg.scale.z = 0.5
		marker_msg.color.g = 1.0
		marker_msg.color.a = 1.0
		marker_msg.header.frame_id = "base_link"

		self.pub_cluster.publish(marker_msg)


	def stop_show_cluster(self):
		marker_msg = Marker()
		marker_msg.header.frame_id = "base_link"
		self.pub_cluster.publish(marker_msg)


	def find_closest_cluster(self):
		closest = (None, 10000)
		for cluster in self.kmeans.cluster_centers_:
			x = cluster[0]
			y = cluster[1]
			distance = math.sqrt(x**2 + y**2)
			if distance < closest[1]:
				closest = (cluster, distance)
		self.closest_cluster = closest
		self.goal_point.x = closest[0][0]
		self.goal_point.y = closest[0][1]


	def drive_to_person(self):
		# figure out how far away person is and angle to person
		self.distance_error = self.closest_cluster[1] - self.goal_distance
		self.angle_error = math.atan2(self.goal_point.y, self.goal_point.x)

		twist = Twist()
		drive_command = self.distance_error * self.distance_k
		turn_command = self.angle_error * self.angle_k
		twist.linear.x = drive_command
		twist.angular.z = turn_command
		self.twist = twist
		
	def stop(self):
		twist = Twist()
		self.pub_cmd_vel.publish(twist)

	def show_points(self):
		marker_msg = Marker()
		marker_msg.type = marker_msg.SPHERE_LIST
		marker_msg.action = marker_msg.ADD
		marker_msg.points = self.convert_points()
		marker_msg.scale.x = 0.05
		marker_msg.scale.y = 0.05
		marker_msg.scale.z = 0.05
		marker_msg.color.b = 1.0
		marker_msg.color.a = 1.0
		marker_msg.header.frame_id = "base_link"

		self.pub_filtered_points.publish(marker_msg)


	def convert_points(self):
		""" takes numpy array of points and converts to an array of Point objects"""
		points = []
		for point in self.scan_points:
			point_object = Point()
			point_object.x = point[0]
			point_object.y = point[1]
			points.append(point_object)
		return points

	def run(self):
		r = rospy.Rate(20)
		while not rospy.is_shutdown():
			self.show_points()
			self.compute_kmeans()
			if self.person_present:
				self.find_closest_cluster()
				self.show_cluster()
				self.drive_to_person()
				self.pub_cmd_vel.publish(self.twist)

			else:
				self.stop()
				self.stop_show_cluster()
			r.sleep()

	def run_finite(self):
		self.show_points()
		self.compute_kmeans()
		if self.person_present:
			self.find_closest_cluster()
			self.show_cluster()
			self.drive_to_person()
		else:
			self.twist = Twist()
			self.stop_show_cluster()
		return self.twist



if __name__ == '__main__':
	node = FollowPerson()
	node.run()
	print("Node closed.")