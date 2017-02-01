#!/usr/bin/env python

""" Independent script that will detect and publish walls """

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy

import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import stats

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



        idxs = sorted(random.sample(range(pts.shape[0]), 2))

        a = pts[idxs[0]]
        b = pts[idxs[1]]

        errorPts = np.asarray([x[1] for x in enumerate(pts) if x[0] not in idxs])

        errors = []
        for c in errorPts:
            ab = b-a
            ac = c-a
            ang = np.arccos(np.dot(ab, ac) / (np.linalg.norm(ab) * np.linalg.norm(ac)))
            dist = np.linalg.norm(ac) * np.sin(ang)
            errors.append([c, dist**2])

        x = []
        y = []
        for pt, dist in errors:
            if dist < .1:
                x.append(pt[0])
                y.append(pt[1])

        rSquared = (stats.linregress(x, y).rvalue)**2
        print("wall r^2", rSquared)

        wallPts = [pt[0] for pt in errors if pt[1] < .1]
        print("wall pts: ", wallPts[0], wallPts[-1])
        print("pts in wall: ", len(wallPts))

        plt.scatter(pts[:,0],pts[:,1],color='blue')
        xy = np.transpose(np.asarray([list(a), list(b)]))
        plt.scatter(xy[0], xy[1],color='red')
        plt.show()

    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()

wall_detector = Wall_Detector()
wall_detector.run()
