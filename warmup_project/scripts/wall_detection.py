#!/usr/bin/env python

""" Independent script that will detect and publish walls """

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy
import numpy as np
import random
import matplotlib.pyplot as plt

class Wall_Detector():

    def __init__(self):
        rospy.init_node('wall_detector')
        rospy.Subscriber('/stable_scan', LaserScan, self.laserCallback)

        self.sleepy = rospy.Rate(2)
        self.angleError = 0


    def laserCallback(self, msg):
        print("laserCallback")

        for ang, dist in enumerate(msg.ranges):
            if dist == 0.0:
                continue
            x = dist * np.cos(ang * np.pi / 180)
            y = dist * np.sin(ang * np.pi / 180)
            try:
                pts = np.append(pts, [[x,y]], axis=0)
            except:
                pts = np.asarray([[x,y]])

        plt.scatter(pts[:,0],pts[:,1])
        plt.show()

        np.random.shuffle(pts)
        a = pts[0]
        b = pts[1]
        print(b-a)

        errors = []

        for c in pts[2:]:
            ab = b-a
            ac = c-a
            ang = np.arccos(np.dot(ab, ac) / (np.linalg.norm(ab) * np.linalg.norm(ac)))
            dist = np.linalg.norm(ac) * np.sin(ang)
            errors.append(dist**2)

        print(errors)
        print(max(errors))
        print(np.mean(errors))
        print(sum(errors))


    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()

wall_detector = Wall_Detector()
wall_detector.run()
