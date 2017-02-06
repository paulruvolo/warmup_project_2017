#!/usr/bin/env python

""" Publishes marker that is a sphere to play with """

from visualization_msgs.msg import Marker
import rospy

pub = rospy.Publisher('cool_sphere', Marker, queue_size = 10)

rospy.init_node('marker_publisher')

r = rospy.Rate(10) #Hz
while not rospy.is_shutdown():

	marker_message = Marker()

	marker_message.type = marker_message.SPHERE
	marker_message.action = marker_message.ADD
	marker_message.pose.position.x = 1.0
	marker_message.pose.position.y = 0.0
	marker_message.pose.position.z = 0.0
	marker_message.scale.x = 0.2
	marker_message.scale.y = 0.2
	marker_message.scale.z = 0.2
	marker_message.color.b = 200
	marker_message.color.a = 1.0
	marker_message.header.frame_id = "base_link"
	marker_message.header.stamp = rospy.Time.now()

	pub.publish(marker_message)

	r.sleep()
