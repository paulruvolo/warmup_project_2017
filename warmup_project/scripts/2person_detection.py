#!/usr/bin/env python

""" Independent script that will detect and publish walls """

from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
import rospy
import tf

import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import stats

class Person_Detection():

    def __init__(self):
        rospy.init_node('person_detector')
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.laserCallback)

        self.listener = tf.TransformListener()

        self.lastCallback = rospy.Time.now()

        self.sleepy = rospy.Rate(5)
        self.angleError = 0

        self.lastScanPts = []


    def laserCallback(self, msg):
        print("laserCallback")
        self.now = rospy.Time.now()
        pts = msg.points

        diff = (self.now - self.lastCallback).to_sec()

        if self.lastScanPts == []:
            self.lastScanPts = pts
            return

        try:
            self.now = rospy.Time.now()
            self.past = self.now - rospy.Duration(.2)
            self.listener.lookupTransform
            self.listener.waitForTransformFull("/odom", self.now,
                                          "/odom", self.past,
                                          "/base_link", rospy.Duration(1.0))
            transformFrame = self.listener.lookupTransformFull("/odom", self.now,
                                          "/odom", self.past,
                                          "/base_link")
            self.lastCallback = self.now

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            print("Exception")
            return

        transformedPts = tf.TransformerROS.transformPointCloud("/base_link", pts)

        try:
            print(transformFrame)
        except:
            pass

        # print pts
        # print self.lastScanPts


    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()


person_detector = Person_Detection()
person_detector.run()
