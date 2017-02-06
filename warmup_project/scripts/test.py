#!/usr/bin/env python

""" Independent script that will detect and publish walls """

from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import PointCloud, LaserScan
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
        # rospy.Subscriber('/projected_stable_scan', PointCloud, self.laserCallback)
        rospy.Subscriber('/stable_scan', LaserScan, self.laserCallback)

        self.lastCallback = rospy.Time.now()

        self.sleepy = rospy.Rate(5)



    def laserCallback(self, msg):
        print("laserCallback")

        self.now = rospy.Time.now()
        print(self.now - self.lastCallback)

        self.lastCallback = self.now


    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()


person_detector = Person_Detection()
person_detector.run()
