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
		x = point*math.cos(angle)
		y = point*math.sin(angle)
		angle += scan.angle_increment
		if(angle<-.5 and abs(x)<4):
			laser_points.append(Point(x,y,0.0))
	publish_line([laser_points[0], laser_points[-1]])

rospy.init_node('wall_detect')
pub = rospy.Publisher('/detect', Marker, queue_size=10)
sub = rospy.Subscriber('/stable_scan', LaserScan, laser_to_cart)
#sub = rospy.Subscriber('/funrobo/laser/scan', LaserScan, laser_to_cart)

def publish_line(line_points):
	marker_message = Marker(id=0, type=Marker.LINE_STRIP,
		header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
		pose=Pose(position=Point(0.0,0.0,0.0)), scale=Vector3(0.1,0.1,0.1),
		color = ColorRGBA(g=1,a=1), points=line_points)
	pub.publish(marker_message)

if __name__ == '__main__':
	rospy.spin()