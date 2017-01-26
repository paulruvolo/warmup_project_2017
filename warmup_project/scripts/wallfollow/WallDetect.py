#!/usr/bin/env python

import rospy, math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose, Point
from std_msgs.msg import Header, ColorRGBA

rospy.init_node('wall_detect')
pub = rospy.Publisher('/detect', Marker, queue_size=10)
sub = rospy.Subscriber('/stable_scan', LaserScan, laser_to_cart)

def laser_to_cart(scan):
	laser_points = []
	angle = scan.angle_min
	for point in scan.ranges:
		#convert to cartesian in here
		
		laser_points.append(Point(x,y,0.0))

	publish_line(laser_points)

def publish_line(laser_points):
	marker_message = Marker(id=0, type=Marker.LINE_STRIP,
		header=Header(stamp=rospy.Time.now(), frame_id='odom'),
		pose=Pose(position=Point(0.0,0.0,0.0), scale=Vector3(1.0,1.0,1.0)),
		points=laser_points)

if __name__ == '__main__':
	rospy.spin()