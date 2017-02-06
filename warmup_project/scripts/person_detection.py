#!/usr/bin/env python

""" Independent script that will detect and publish walls """

from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from itertools import cycle
import rospy
import tf
from sklearn.cluster import MeanShift, estimate_bandwidth

import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import stats

class Person_Detection():

    def __init__(self):
        rospy.init_node('person_detector')
        rospy.Subscriber('/stable_scan', LaserScan, self.laserCallback)

        self.sleepy = rospy.Rate(5)

        self.lastScanPts = []


    def laserMsgToPts(self, msg):
        """ Uses self.ms to generate a list of points """


        # self.marker.header = msg.header

        for ang, dist in enumerate(msg.ranges[0:360]):
            ang = ang + 180
            if dist == 0.0:
                continue
            x = dist * np.cos(ang * np.pi / 180)
            y = dist * np.sin(ang * np.pi / 180)
            try:
                pts = np.append(pts, [[x,y]], axis=0)
            except:
                pts = np.asarray([[x,y]])
        return (pts)

    def laserCallback(self, msg):
        print("laserCallback")
        X = self.laserMsgToPts(msg)


        # The following bandwidth can be automatically detected using
        bandwidth = estimate_bandwidth(X, quantile=0.2, n_samples=500)
        ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
        ms.fit(X)
        labels = ms.labels_
        cluster_centers = ms.cluster_centers_

        labels_unique = np.unique(labels)
        n_clusters_ = len(labels_unique)

        print("number of estimated clusters : %d" % n_clusters_)

        plt.figure(1)
        plt.clf()

        colors = cycle('bgrcmykbgrcmykbgrcmykbgrcmyk')
        for k, col in zip(range(n_clusters_), colors):
            my_members = labels == k
            cluster_center = cluster_centers[k]
            plt.plot(X[my_members, 0], X[my_members, 1], col + '.')
            plt.plot(cluster_center[0], cluster_center[1], 'o', markerfacecolor=col,
                     markeredgecolor='k', markersize=14)
        plt.title('Estimated number of clusters: %d' % n_clusters_)
        plt.show()



    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()


person_detector = Person_Detection()
person_detector.run()
