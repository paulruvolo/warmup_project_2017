#!/usr/bin/env python

""" Determines the relative angle of a wall off to the right and matches it """

import rospy
import math
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

class FollowWall(object):
	def __init__(self):
		rospy.init_node('wall_follow')
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.wall_pub = rospy.Publisher('detected_wall', Marker, queue_size=10)
		self.twist = Twist()

		self.angle_k = 1.8
		self.angle_error = None

		self.l1 = 1.0
		self.l2 = 1.0

	def process_scan(self, msg):
		l1 = msg.ranges[315]
		if l1 != 0.0:
			self.l1 = l1
		l2 = msg.ranges[225]
		if l2 != 0.0:
			self.l2 = l2

	def calculate_angle_error(self):
		self.angle_error = -self.angle_diff(math.pi/4, math.atan2(self.l2, self.l1))

	def publish_wall(self):
		wall = Marker()
		wall.type = wall.CUBE
		wall.action = wall.ADD
		wall.pose.position.x = -math.sqrt(2)/2*self.l2
		wall.pose.position.y = -math.sqrt(2)/2*self.l2
		wall.scale.x = 10
		wall.scale.y = 0.05
		wall.scale.z = 0.5
		wall.color.b = 200
		wall.color.a = 1.0
		wall.header.frame_id = "base_link"
		wall.header.stamp = rospy.Time.now()

		quaternion_angles = quaternion_from_euler(0,0,self.angle_error)
		wall.pose.orientation.x = quaternion_angles[0]
		wall.pose.orientation.y = quaternion_angles[1]
		wall.pose.orientation.z = quaternion_angles[2]
		wall.pose.orientation.w = quaternion_angles[3]

		self.wall_pub.publish(wall)


	def find_twist(self):
		twist = Twist()
		turn_command = self.angle_error * self.angle_k
		drive_command = 0.8
		twist.linear.x = drive_command
		twist.angular.z = turn_command
		return twist

	def angle_diff(self, a, b):
		a = self.angle_normalize(a)
		b = self.angle_normalize(b)
		d1 = a-b
		d2 = 2*math.pi - math.fabs(d1)
		if d1>0:
			d2 *= -1.0
		if (math.fabs(d1)<math.fabs(d2)):
			return d1
		else:
			return d2

	def angle_normalize(self, z):
		return math.atan2(math.sin(z),math.cos(z))

	def drive_parallel(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.calculate_angle_error()
			self.publish_wall()
			twist = self.find_twist()
			self.pub.publish(twist)
			r.sleep()

	def stop(self):
		twist = Twist()
		self.pub.publish(twist)

	def drive_finite(self):
		self.calculate_angle_error()
		self.publish_wall()
		twist = self.find_twist()
		self.twist = twist

if __name__ == '__main__':
	node = FollowWall()
	node.drive_parallel()
	node.stop()
	print("Node closed")


