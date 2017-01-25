#!/usr/bin/env python

""" Publishes marker that is a sphere to play with """

from visualization_msgs.msg import Marker
import rospy

pub = rospy.Publisher('cool_sphere', Marker, queue_size = 5)

rospy.init_node('marker_publisher')

r = rospy.Rate(10) #Hz
while not rospy.is_shutdown():

	#marker


	r.sleep()
