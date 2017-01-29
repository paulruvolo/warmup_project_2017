#!/usr/bin/env python

import rospy

from visualization_msgs.msg import Marker

marker = Marker()
marker.header.frame_id = "/neck"
marker.type = marker.SPHERE
marker.action = marker.ADD
marker.scale.x = 1
marker.scale.y = 1
marker.scale.z = 1
marker.color.a = 1.0
marker.pose.orientation.w = 1.0
marker.pose.position.x = 1.0
marker.pose.position.y = 2.0
marker.pose.position.z = 0.0

rospy.init_node('my_marker')
pub = rospy.Publisher('/marker',Marker,queue_size=10)
r = rospy.Rate(10)




while not rospy.is_shutdown():

	r.sleep()
	pub.publish(marker)
