#!/usr/bin/env python

import rospy
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3, Pose
from std_msgs.msg import Header, ColorRGBA

rospy.init_node('test_wall_follower')
time.sleep(.5)

pub = rospy.Publisher('/detect', Marker, queue_size=10, latch=True)



def publish_line(laser_points):
    marker_message = Marker(id=0, type=Marker.LINE_STRIP,
        header=Header(stamp=rospy.Time.now() - rospy.Duration(secs=2), frame_id='odom'),
        pose=Pose(position=Point(0.0,0.0,0.0)), scale=Vector3(.1, .1, .1),
        color=ColorRGBA(g=1, a=1),
        points=laser_points)

    pub.publish(marker_message)

# Happy and go straight
point1 = Point(x=0, y=-1.5)
point2 = Point(x=5, y=-1.5)


r = rospy.Rate(100)
while not rospy.is_shutdown():
    r.sleep()
    publish_line([point1, point2])


