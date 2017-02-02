#! /usr/bin/env python

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

from visualization_msgs.msg import Marker


class Person_Follower():

    def __init__(self):
        rospy.init_node('person_follow')

        self.mark = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.processScans, queue_size=10)
        self.current_error = None
        self.target_distance = rospy.get_param('~target_distance',1)
        self.target_angle = 0
        self.k_lin = rospy.get_param('~k_lin', .5)
        self.k_ang = rospy.get_param('~k_ang', .5)
        self.raw_feet_scans = []
        self.filtered_feet_scans = []
        self.angles = range(30)
        self.angles.extend(range(330,360))
        self.centroid = None

        self.max_scans = 8
        self.past_centroid = (0,1)

        self.init_marker()


    def init_marker(self):
        self.marker = Marker()
        self.marker.type = 2
        self.marker.header.frame_id = "base_laser_link"
        self.marker.scale.x = .1
        self.marker.scale.y = .1
        self.marker.scale.z = .1
        self.marker.color.a = 1
        self.marker.color.r = 0
        self.marker.color.g = 1
        self.marker.color.b = 1

    def processScans(self, msg):
        self.raw_feet_scans = []
        self.filtered_feet_scans = []
        for i in self.angles:
            if msg.ranges[i] > 0.0 and msg.ranges[i] < 3.0:
                if i > 180:
                    self.raw_feet_scans.append((i-360, msg.ranges[i]))
                else:
                    self.raw_feet_scans.append((i, msg.ranges[i]))

        past_x = self.past_centroid[1] * math.cos(math.radians(self.past_centroid[0]))
        past_y = self.past_centroid[1] * math.sin(math.radians(self.past_centroid[0]))

        for point in self.raw_feet_scans:
            current_x = point[1] * math.cos(math.radians(point[0]))
            current_y = point[1] * math.sin(math.radians(point[0]))
            self.filtered_feet_scans.append((point[0], point[1], np.sqrt(np.square(current_x - past_x) + np.square(current_y - past_y))))

        self.filtered_feet_scans = sorted(self.filtered_feet_scans, key=lambda tup: tup[2])

        if len(self.filtered_feet_scans) > self.max_scans:
            self.filtered_feet_scans = self.filtered_feet_scans[:self.max_scans]

        self.centroid = (np.mean([item[0] for item in self.filtered_feet_scans]), np.mean([item[1] for item in self.filtered_feet_scans]))

        self.past_centroid = self.centroid

        #Marker stuff
        point1 = self.centroid[1] * math.cos(math.radians(self.centroid[0]))
        point2 = self.centroid[1] * math.sin(math.radians(self.centroid[0]))

        self.marker.pose.position.x = -point1
        self.marker.pose.position.y = -point2
        self.marker.pose.position.z = 0
        self.mark.publish(self.marker)

    def act(self):
        twist_msg = Twist()
        if self.centroid:
            lin_error = self.centroid[1]*math.cos(math.radians(self.centroid[0])) - self.target_distance
            ang_error = self.target_angle - self.centroid[0]
            twist_msg.linear.x = self.k_lin * lin_error
            twist_msg.angular.z = -self.k_ang * math.radians(ang_error)

        self.pub.publish(twist_msg)




if (__name__=="__main__"):
    peoples = Person_Follower()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()

        peoples.act()
