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

		self.angle_k = 1
		self.position_k = 1
		self.drive_angle_k = 1

		self.position = Point()
		self.goal_position = Point()
		self.position_error = None
		self.angle = -10
		self.goal_angle = 0.0
		self.angle_error = None

		self.at_goal_angle = False
		self.at_point = False
	

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
		turn_command = self.angle_error * self.angle_k
		print "turn command: " + str(turn_command)
		twist.angular.z = turn_command
		return twist

	def calculate_position_error(self):
		delta_x = self.goal_position.x - self.position.x
		delta_y = self.goal_position.y - self.position.y
		self.position_error = math.sqrt((delta_x)**2 + (delta_y)**2)
		self.goal_angle = math.atan2(delta_y, delta_x)
		self.angle_error = -self.angle_diff(self.angle,self.goal_angle)
		if math.fabs(self.position_error) < 0.05:
			return True
		else:
			return False


	def find_position_twist(self):
		twist = Twist()
		drive_command = self.position_error * self.position_k
		turn_command = self.angle_error * self.drive_angle_k
		twist.linear.x = drive_command
		twist.angular.z = turn_command
		return twist

	def turn_in_place(self):
		self.at_goal_angle = self.calculate_angle_error()
		if self.at_goal_angle:
			self.stop()
		else:
			twist = self.find_angle_twist()
			self.pub.publish(twist)

	def drive_to_point(self):
		self.at_point = self.calculate_position_error()
		if self.at_point:
			self.stop()
		else:
			twist = self.find_position_twist()
			self.pub.publish(twist)

	def stop(self):
		twist = Twist()
		self.pub.publish(twist)


	def run_angle(self, goal):
		r = rospy.Rate(10)
		self.goal_angle = goal

		while not rospy.is_shutdown():	
			if not self.at_goal_angle:
				self.turn_in_place()	
			else:
				break
			r.sleep()
		self.at_goal_angle = False
		print "Reached Angle!"

	def run_point(self, goal_x, goal_y):
		r = rospy.Rate(10)
		goal_point = Point()
		goal_point.x = goal_x
		goal_point.y = goal_y
		self.goal_position = goal_point

		while not rospy.is_shutdown():	
			if not self.at_point:
				self.drive_to_point()	
			else:
				break
			r.sleep()
		self.at_point = False
		print "Reached Point!"

			

if __name__ == '__main__':
	side = 1
	node = DriveSquare()
	node.run_angle(0*math.pi/2)
	node.run_point(side,0)
	node.run_angle(1*math.pi/2)
	node.run_point(side,side)
	node.run_angle(2*math.pi/2)
	node.run_point(0,side)
	node.run_angle(3*math.pi/2)
	node.run_point(0,0)
	print("Finished instructions.")

