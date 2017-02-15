#! /usr/bin/env python
"""
person_follower.py
Kevin Zhand and Shane Kelly
CompRobo Spring 2017

This script allows the Neato to follow a person around an area using proportional control and shifted means.
Specifically, it will follow the legs of a person, so long as the person doesn't move too erratically.
"""

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

from visualization_msgs.msg import Marker


class Person_Follower():
    """
    The object that holds the ROS nodes and contains the person following functionality
    Contains an act() mehtod that moves the Neato based on what is received and calculated in processScans
    """

    def __init__(self):
        #Init ROS things
        rospy.init_node('person_follow')
        self.mark = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.processScans, queue_size=10)

        #Proportional control
        self.current_error = None
        self.target_distance = rospy.get_param('~target_distance',1)
        self.target_angle = 0
        self.k_lin = rospy.get_param('~k_lin', .5)
        self.k_ang = rospy.get_param('~k_ang', .5)

        #Shifted Means variables
        self.raw_feet_scans = []
        self.filtered_feet_scans = []
        self.angles = range(30)
        self.angles.extend(range(330,360))
        self.centroid = None
        self.max_scans = 8
        self.past_centroid = (0,1)

        #Init markers
        self.init_marker()


    def init_marker(self):
        """
        Creates a marker to visualize the point that the Neato is trying to move to
        Defined as the centroid
        """

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
        """
        Callback for LIDAR data from /stable_scan
        Takes in msg, which is a LaserScan data type that holds a variable called ranges that holds 361 datapoints,
        which correspond to the laser scans from degree 0-360 inclusive

        Uses the range data to fill raw_feet_scans, then create filtered_feet_scans from raw_feet_scans by selecting the 8
        closest data points to the centroid determined in the previous iteration
        Then updates the two centroids.
        """

        #Resets the two lists
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

        #First populates the filtered_feet_scans
        for point in self.raw_feet_scans:
            current_x = point[1] * math.cos(math.radians(point[0]))
            current_y = point[1] * math.sin(math.radians(point[0]))
            self.filtered_feet_scans.append((point[0], point[1], np.sqrt(np.square(current_x - past_x) + np.square(current_y - past_y))))

        #Then sorts filtered_feet_scans and truncates the list at the first 8
        self.filtered_feet_scans = sorted(self.filtered_feet_scans, key=lambda tup: tup[2])
        if len(self.filtered_feet_scans) > self.max_scans:
            self.filtered_feet_scans = self.filtered_feet_scans[:self.max_scans]

        #Updates centroids
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
        """
        Calculates the next movement of the Neato in Twist msg using proportional control
        The objective is to try and minimize the difference between the current centroid and the target distance to 0
        as well as the minimize the angle of the Neato towards the target to 0

        Publishes to /cmd_vel the Twist msg which contains linear and angular velocity for the Neato to move
        """

        twist_msg = Twist()
        if self.centroid:
            lin_error = self.centroid[1]*math.cos(math.radians(self.centroid[0])) - self.target_distance
            ang_error = self.target_angle - self.centroid[0]
            twist_msg.linear.x = self.k_lin * lin_error
            twist_msg.angular.z = -self.k_ang * math.radians(ang_error)

        self.pub.publish(twist_msg)




if (__name__=="__main__"):
    """
    Main method, which initializes and runs the Neato on Person_follower()
    """

    peoples = Person_Follower()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()

        peoples.act()
