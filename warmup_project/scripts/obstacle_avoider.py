#!usr/bin/env python

""" avoids obstacels while driving in a desired direction """

import rospy
import math
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Point, PointStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

class AvoidObstacle(object):
	def __init__(self):
		rospy.init_node('obstacle_avoid')
		rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
		self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
		self.pub_filtered_points = rospy.Publisher('/filtered_points', Marker, queue_size=10)
		self.pub_total_force = rospy.Publisher('/total_force', Marker, queue_size=10)
		self.pub_propel_force = rospy.Publisher('/propel_force', Marker, queue_size=10)
		self.pub_goal_point = rospy.Publisher('/goal_point', Marker, queue_size=10)
		self.pub_drive_force = rospy.Publisher('/drive_force', Marker, queue_size=10)
		self.tf_listener = tf.TransformListener()

		self.obstacle_distance = 1.2
		self.scan_points = []
		self.goal_point = PointStamped(point = Point(x=-100, y=0), header = Header(frame_id="odom") )
		
		self.attraction_proportion = 0.01 # how much the robot cares about the goal, relative to its obstacles
		self.angle_k = 1
		self.linear_k = 0.08

		self.propel_force = (0,0) # cartesian, in base_link (x, y)
		
		self.total_force = (0,0) # polar, in base_link (phi, rho)

		self.cart_weighted_total_force = (0,0)
		self.cart_weighted_propel_force = (0,0)

		self.drive_force = (0,0)

	def process_scan(self, msg):
		points = []
		for angle,distance in enumerate(msg.ranges):
			if (distance < self.obstacle_distance) and (distance > 0):
				points.append((angle, distance))
		self.scan_points = points


	def cart2pol(self, x, y):
	    rho = math.sqrt(x**2 + y**2)
	    phi = math.atan2(y, x)
	    return(phi, rho)

	def pol2cart(self,theta,r):
		x = r * math.cos(theta)
		y = r * math.sin(theta)
		return (x,y)

	def sum_force(self):
		total_force = (0,0)
		try:
			transformed_goal_point = self.tf_listener.transformPoint('/base_link', self.goal_point)
			self.propel_force = self.cart2pol(transformed_goal_point.point.x, transformed_goal_point.point.y)
		except:
			pass
		
		for point in self.scan_points:
			theta = point[0]
			r = -0.05 / point[1]**2
			point_force = self.pol2cart(math.radians(theta),r) # angle, -1/distance
			#sum the x and y components of the force
			total_force = (total_force[0] + point_force[0], total_force[1] + point_force[1])

		self.total_force = self.cart2pol(total_force[0], total_force[1])


	def convert_total_force_to_points(self):
		start_point = Point(x=0, y=0)
		end_point = Point()
		end_point.x = self.cart_weighted_total_force[0]
		end_point.y = self.cart_weighted_total_force[1]
		return [start_point, end_point]

	def show_total_force(self):
		marker_msg = Marker()
		marker_msg.type = marker_msg.ARROW
		marker_msg.action = marker_msg.ADD
		marker_msg.points = self.convert_total_force_to_points()
		marker_msg.scale.x = 0.05
		marker_msg.scale.y = 0.1
		marker_msg.color.b = 1.0
		marker_msg.color.a = 1.0
		marker_msg.header.frame_id = "base_link"
		marker_msg.header.stamp = rospy.Time.now()

		self.pub_total_force.publish(marker_msg)

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

	def show_goal_point(self):
		marker_msg = Marker()
		marker_msg.type = marker_msg.SPHERE
		marker_msg.action = marker_msg.ADD
		marker_msg.pose.position.x = self.goal_point.point.x
		marker_msg.pose.position.y = self.goal_point.point.y
		marker_msg.scale.x = 0.2
		marker_msg.scale.y = 0.2
		marker_msg.scale.z = 0.2
		marker_msg.color.r = 1.0
		marker_msg.color.a = 1.0
		marker_msg.header.frame_id = "odom"

		self.pub_goal_point.publish(marker_msg)

	def show_propel_force(self):
		marker_msg = Marker()
		marker_msg.type = marker_msg.ARROW
		marker_msg.action = marker_msg.ADD
		marker_msg.points = [Point(x=0, y=0), Point(x=self.cart_weighted_propel_force[0], y=self.cart_weighted_propel_force[1])]
		marker_msg.scale.x = 0.05
		marker_msg.scale.y = 0.1
		marker_msg.color.r = 1.0
		marker_msg.color.a = 1.0
		marker_msg.header.frame_id = "base_link"
		marker_msg.header.stamp = rospy.Time.now()

		self.pub_propel_force.publish(marker_msg)

	def show_drive_force(self):
		marker_msg = Marker()
		marker_msg.type = marker_msg.ARROW
		marker_msg.action = marker_msg.ADD
		cart_drive_force = self.pol2cart(self.drive_force[0], self.drive_force[1])
		marker_msg.points = [Point(x=0, y=0), Point(x=cart_drive_force[0], y=cart_drive_force[1])]
		marker_msg.scale.x = 0.05
		marker_msg.scale.y = 0.1
		marker_msg.color.g = 1.0
		marker_msg.color.a = 1.0
		marker_msg.header.frame_id = "base_link"
		marker_msg.header.stamp = rospy.Time.now()

		self.pub_drive_force.publish(marker_msg)

	def convert_points(self):
		points = []
		for point in self.scan_points:
			point_object = Point()
			cart_point = self.pol2cart(math.radians(point[0]), point[1])
			point_object.x = cart_point[0]
			point_object.y = cart_point[1]
			points.append(point_object)
		return points

	def stop(self):
		twist = Twist()
		self.pub_cmd_vel.publish(twist)

	def drive(self):
		twist = Twist()
		weighted_total_force = (self.total_force[0], self.total_force[1] * (1-self.attraction_proportion))
		weighted_propel_force = (self.propel_force[0], self.propel_force[1] * self.attraction_proportion)

		self.cart_weighted_total_force = self.pol2cart(weighted_total_force[0], weighted_total_force[1])
		self.cart_weighted_propel_force = self.pol2cart(weighted_propel_force[0], weighted_propel_force[1])

		self.drive_force = self.cart2pol(self.cart_weighted_total_force[0] + self.cart_weighted_propel_force[0], self.cart_weighted_total_force[1] + self.cart_weighted_propel_force[1])

		drive_command = self.drive_force[1] * self.linear_k
		turn_command = self.drive_force[0] * self.angle_k

		twist.linear.x = drive_command
		twist.angular.z = turn_command
		self.pub_cmd_vel.publish(twist)

	def run(self):
		r = rospy.Rate(20)
		while not rospy.is_shutdown():
			self.sum_force()
			self.show_total_force()
			self.show_points()
			self.show_propel_force()
			self.show_goal_point()
			self.show_drive_force()
			self.drive()
			r.sleep()

if __name__ == '__main__':
	node = AvoidObstacle()
	node.run()
	print("Node closed.")