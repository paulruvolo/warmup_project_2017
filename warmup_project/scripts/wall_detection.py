#!/usr/bin/env python

""" Independent script that will detect and publish walls """

from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import rospy

import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import stats

class Wall_Detector():

    def __init__(self):
        rospy.init_node('wall_detector')
        rospy.Subscriber('/stable_scan', LaserScan, self.laserCallback)
        self.marker_pub = rospy.Publisher('/marker_pub',Marker,queue_size=10)

        self.sleepy = rospy.Rate(5)
        self.angleError = 0

        """ Start Point """
        self.start_point = Point()
        self.start_point.x = 1.0
        self.start_point.y = 0.0
        self.start_point.z = 0.0

        """ End Point """
        self.end_point = Point()
        self.end_point.x = 0.0
        self.end_point.y = 1.0
        self.end_point.z = 0.0

        """ Marker Init """
        self.marker = Marker()
        self.marker.header.frame_id = "/odom"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = 0
        self.marker.scale.x = 0.02
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.points.append(self.start_point)
        self.marker.points.append(self.end_point)

        """ Debug Attributes """
        self.xy = None

    def laserMsgToPts(self, msg):
        """ Uses self.ms to generate a list of points """
        for ang, dist in enumerate(msg.ranges[0:360]):
            if dist == 0.0:
                continue
            x = dist * np.cos(ang * np.pi / 180)
            y = dist * np.sin(ang * np.pi / 180)
            try:
                pts = np.append(pts, [[x,y]], axis=0)
            except:
                pts = np.asarray([[x,y]])
        return (pts)


    def getDistErrors(self, pts):
        """ Returns the distance errors from two sampled random points """
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

        self.xy = np.transpose(np.asarray([list(a), list(b)]))
        return (errors)


    def getWallPoints(self, errors):
        """ Returns the points of the wall given list of errors """
        wallPts = [pt[0] for pt in errors if pt[1] < .1]
        return (wallPts)


    def getEndPoints(self, wallPts):
        """ Returns the end points given a list of wall points"""
        endPt1 = wallPts[0]
        endPt2 = wallPts[1]
        for pt in wallPts[2:]:
            dist = np.linalg.norm(endPt1 - endPt2)
            if np.linalg.norm(endPt1 - pt) > dist:
                endPt2 = pt
            elif np.linalg.norm(pt - endPt2) > dist:
                endPt1 = pt
        return (endPt1, endPt2)


    def getRSquared(self, wallPts):
        """ Retunrs the r^2 given a list of wallPts """
        rSquared = (stats.linregress(np.transpose(np.asarray(wallPts))).rvalue)**2
        return (rSquared)


    def laserCallback(self, msg):
        print("laserCallback")

        pts = self.laserMsgToPts(msg)

        rSquared = 0.0
        maxRSquared = .95

        while rSquared < maxRSquared:
            maxRSquared -= .025

            errors = self.getDistErrors(pts)
            wallPts = self.getWallPoints(errors)
            endPt1, endPt2 = self.getEndPoints(wallPts)
            rSquared = self.getRSquared(wallPts)

            print("wall pts: ", endPt1, endPt2)
            print("pts in wall: ", len(wallPts))
            print("wall r^2", rSquared)

        if rSquared > .90:
            self.start_point.x = endPt1[0]
            self.start_point.y = endPt1[1]
            self.end_point.x   = endPt2[0]
            self.end_point.y   = endPt2[1]
            self.publishMarker()

        plt.scatter(pts[:,0],pts[:,1],color='blue')
        plt.scatter(self.xy[0], self.xy[1],color='red')
        endPts = np.transpose(np.concatenate((endPt1, endPt2)).reshape((2,2)))
        plt.scatter(endPts[0], endPts[1], color='yellow')
        plt.show()


    def publishMarker(self):
        self.marker_pub.publish(self.marker)


    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()


wall_detector = Wall_Detector()
wall_detector.run()
