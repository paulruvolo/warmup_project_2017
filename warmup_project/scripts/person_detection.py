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




        self.sleepy = rospy.Rate(5)
        self.angleError = 0

        self.lastScanPts = []


    def laserCallback(self, msg):
        print("laserCallback")
        pts = msg.points

        if self.lastScanPts == []:
            self.lastScanPts = pts
            return

        try:
            self.now = rospy.Time.now()
            self.past = self.now - rospy.Duration(.2)
            self.listener.waitForTransformFull("/odom", self.now,
                                          "/odom", self.past,
                                          "/base_link", rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransformFull("/odom", self.now,
                                          "/odom", self.past,
                                          "/base_link")

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            print("Exception")
            return

        try:
            print(trans, rot)
        except:
            pass

        # print pts
        # print self.lastScanPts


    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()


person_detector = Person_Detection()
person_detector.run()
