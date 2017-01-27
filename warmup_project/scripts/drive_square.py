#!/usr/bin/env python

""" Uses Odometry to drive the robot in a square """

import rospy
import math
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class DriveSquare(object):
	def __init__(self):
		rospy.init_node('drive_square')
		rospy.Subscriber('/odom', Odometry, self.process_odom)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.angle_proportional_constant = 1

		self.position = Point()
		self.goal_position = Point()
		self.angle = -10
		self.goal_angle = 0.0
		self.angle_error = None
	

	def process_odom(self, msg):
		self.position.x = msg.pose.pose.position.x
		self.position.y = msg.pose.pose.position.y
		orientation_tuple = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		self.angle = angles[2]

	def calculate_angle_error(self):
		self.angle_error = -self.angle_diff(self.angle, self.goal_angle)
		print "angle error: " + str(self.angle_error) + "   angle: " + str(self.angle) + "   goal: " + str(self.goal_angle)
		if math.fabs(self.angle_error) < 0.05:
			return True
		else:
			return False

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


	def find_angle_twist(self):
		twist = Twist()
		turn_command = self.angle_error * self.angle_proportional_constant
		print "turn command: " + str(turn_command)
		twist.angular.z = turn_command
		return twist


	def run(self):
		r = rospy.Rate(10)
		at_goal_angle = False
		self.goal_angle = 1*math.pi/2

		while not rospy.is_shutdown():
			#define sqaure points based on pose
			#define angles to turn to after each point
			#for point,angle in goals
			#	set point as goal 
			#	while not at_position_goal
			#		#calculate error 
			#		#update twist
			#		#publish
			#		r.sleep()
			#	while not at_angle_goal
			#		#calculate error
			#		#update twist
			#		#publish
			#		r.sleep()
			#stop
			#publish stop
			#celebrate!

			if not at_goal_angle:
				at_goal_angle = self.calculate_angle_error()
				if at_goal_angle:
					twist = Twist()
					twist.angular.z = 0
				else:
					twist = self.find_angle_twist()
				self.pub.publish(twist)
			else:
				break
			r.sleep()
		print "Done!"

			

if __name__ == '__main__':
	node = DriveSquare()
	node.run()


