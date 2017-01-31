#!/usr/bin/env python

""" Independent script that will detect and publish walls """

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy
import numpy as np

class Wall_Detector():

    def __init__(self):
        rospy.init_node('wall_detector')
        rospy.Subscriber('/scan', LaserScan, self.laserCallback)

        self.sleepy = rospy.Rate(2)
        self.angleError = 0


    def laserCallback(self, msg):
        print("laserCallback")

        # convert scan data to list of points
        pts = []

        for ang, dist in enumerate(msg.ranges):
            if dist == 0.0:
                continue
            x = dist * np.cos(ang)
            y = dist * np.sin(ang)
            pts.append((x, y))

        print(np.random.choice(pts, 2, replace=False))


    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()

wall_detector = Wall_Detector()
wall_detector.run()
