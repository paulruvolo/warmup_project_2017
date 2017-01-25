#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

class MarkerPub():

    def __init__(self):
        rospy.init_node('marker')
        self.pub = rospy.Publisher('/visualization_marker', Marker,
                                   queue_size=10)

if (__name__=="__main__"):
    marker_pub = MarkerPub()
    marker = Marker()
    marker.pose.position.x = 1
    marker.pose.position.y = 2
    marker.header.frame_id = "base_link"
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 1
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        marker_pub.pub.publish(marker)
