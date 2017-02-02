#! /usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

class Person_Follower():

    def __init__(self):
        rospy.init_node('person_follow')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.processScans, queue_size=10)
        self.current_error = None
        self.target_distance = rospy.get_param('~target_distance',1)
        self.target_angle = 0
        self.k = rospy.get_param('~k', .5)
        self.feet_scans = []
        self.angles = range(15)
        self.angles.extend(range(345,360))
        self.centroid = None

    def processScans(self, msg):
        for i in self.angles:
            if msg.ranges[i] > 0.0 and msg.ranges[i] < 3.0:
                if i > 180:
                    self.feet_scans.append((i-360, msg.ranges[i]))
                else:
                    self.feet_scans.append((i, msg.ranges[i]))
        self.centroid = (float(sum([item(0) for item in self.feet_scans]))/len(self.feet_scans), float(sum([item(1) for item in self.feet_scans]))/len(self.feet_scans))

    def act(self):
        twist_msg = Twist()
        lin_error = self.centroid(1)*math.cos(math.radians(self.centroid(0))) - self.target_distance
        ang_error = self.target_angle - self.centroid(0)
        twist_msg.linear.x = self.k * lin_error
        twist_msg.angular.z = self.k * ang_error

        self.pub.publish(twist_msg)




if (__name__=="__main__"):
    peoples = Person_Follower()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        peoples.act()
