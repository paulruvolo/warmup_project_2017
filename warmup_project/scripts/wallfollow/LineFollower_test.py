#!/usr/bin/env python

import rospy
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3, Pose
from std_msgs.msg import Header, ColorRGBA

rospy.init_node('test_wall_follower')
time.sleep(.5)

pub = rospy.Publisher('/detect', Marker, queue_size=10)


def publish_line(laser_points):
    marker_message = Marker(id=0, type=Marker.LINE_STRIP,
        header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
        pose=Pose(position=Point(0.0,0.0,0.0)), scale=Vector3(.1, .1, .1),
        color=ColorRGBA(g=1, a=1),
        points=laser_points)

    pub.publish(marker_message)

# Happy and go straight
point1 = Point(x=0, y=1)
point2 = Point(x=5, y=1)

publish_line([point1, point2])

while not rospy.is_shutdown():
    time.sleep(1)


