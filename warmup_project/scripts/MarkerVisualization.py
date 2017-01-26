#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point
from std_msgs.msg import Header, ColorRGBA

rospy.init_node('marker_node')
pub = rospy.Publisher('/marker', Marker, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
	marker_message = Marker(header=Header(stamp=rospy.Time.now(), frame_id='odom'),
		pose=Pose(position=Point(1.0,2.0,0.0)), scale=Vector3(1.0,1.0,1.0),
		color=ColorRGBA(.1,0.0,.9,1.0), id=1, type=Marker.SPHERE)
	pub.publish(marker_message)
	r.sleep()