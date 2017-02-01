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

        for ang, dist in enumerate(msg.ranges[0:360]):
            if dist == 0.0:
                continue
            x = dist * np.cos(ang * np.pi / 180)
            y = dist * np.sin(ang * np.pi / 180)
            try:
                pts = np.append(pts, [[x,y]], axis=0)
            except:
                pts = np.asarray([[x,y]])


        rSquared = 0.0
        maxRSquared = .95

        while rSquared < maxRSquared:
            maxRSquared -= .01

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

            wallPts = [pt[0] for pt in errors if pt[1] < .1]
            print("wall pts: ", wallPts[0], wallPts[-1])
            print("pts in wall: ", len(wallPts))

            rSquared = (stats.linregress(np.transpose(np.asarray(wallPts))).rvalue)**2
            print("wall r^2", rSquared)

        plt.scatter(pts[:,0],pts[:,1],color='blue')
        xy = np.transpose(np.asarray([list(a), list(b)]))
        plt.scatter(xy[0], xy[1],color='red')
        endPts = np.transpose(np.concatenate((wallPts[0], wallPts[-1])).reshape((2,2)))
        plt.scatter(endPts[0], endPts[1], color='yellow')
        plt.show()

    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()

wall_detector = Wall_Detector()
wall_detector.run()
