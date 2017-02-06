#!/usr/bin/env python

""" Independent script that will detect and publish walls """

from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from itertools import cycle
import rospy
import tf
from nav_msgs.msg import Odometry
import math
from sklearn.cluster import MeanShift, estimate_bandwidth
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix


import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import stats

class Obstacle_Detector():

    def __init__(self):
        rospy.init_node('obstacle_detector')
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.laserCallback)
        rospy.Subscriber("/odom",Odometry,self.process_odom)
        self.point_pub = rospy.Publisher('/clear_path_point',Point,queue_size=10)
        self.marker_pub = rospy.Publisher('/clear_path',Marker,queue_size=10)


        self.point = Point()
        self.point.x = 0
        self.point.y = 0
        self.point.z = 0

        self.sleepy = rospy.Rate(5)

        self.lastScanPts = []

        #current location and orientation
        self.currentx = 0.0
        self.currenty = 0.0
        self.orientation = 0.0

        """ Marker Init """
        self.marker = Marker()
        self.marker.header.frame_id = "/odom"
        self.marker.type = self.marker.SPHERE
        self.marker.action = 0
        self.marker.scale.x = .3
        self.marker.scale.y = .3
        self.marker.scale.z = .3
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.pose.position.x = self.point.x
        self.marker.pose.position.y = self.point.y
        self.marker.pose.position.z = self.point.z


    def process_odom(self,msg):
        orientation_tuple = (msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        self.currentx = msg.pose.pose.position.x
        self.currenty = msg.pose.pose.position.y
        self.orientation = angles[2]


    def angle_normalize(self,z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))


    def angle_diff(self,a,b):
        """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
        """
        self.a = self.angle_normalize(a)
        self.b = self.angle_normalize(b)
        self.d1 = a-b
        self.d2 = 2*math.pi - math.fabs(self.d1)
        if self.d1 > 0:
            self.d2 *= -1.0
        if math.fabs(self.d1) < math.fabs(self.d2):
            return self.d1
        else:
            return self.d2


    def laserCallback(self, msg):
        print("laserCallback")
        X = []
        for pt in msg.points:
            try:
                X = np.append(X, [[pt.x,pt.y]], axis=0)
            except:
                X = np.asarray([[pt.x,pt.y]])


        # The following bandwidth can be automatically detected using
        bandwidth = estimate_bandwidth(X, quantile=0.2, n_samples=500)
        ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
        ms.fit(X)
        labels = ms.labels_
        cluster_centers = ms.cluster_centers_


        vectors = []
        for targetx, targety in cluster_centers:
            dist = -1.0 / np.linalg.norm([targetx-self.currentx, targety-self.currenty])
            self.targetangle = math.atan2(targety-self.currenty,targetx-self.currentx)
            self.angledifference  = self.angle_diff(self.targetangle,self.orientation)
            vector = [dist * math.cos(self.angledifference),
                      dist * math.sin(self.angledifference)]
            vectors.append(vector)

        print(vectors)

        newVector = np.sum(np.asarray(vectors), axis=0)
        newVector = newVector / np.linalg.norm(newVector)
        print newVector

        #
        # print(smallestDistancePt)

        self.point.x, self.point.y = newVector
        self.point_pub.publish(self.point)

        ## update maker position
        self.marker.pose.position.x = self.point.x
        self.marker.pose.position.y = self.point.y
        self.marker.pose.position.z = self.point.z
        self.marker_pub.publish(self.marker)

        # labels_unique = np.unique(labels)
        # n_clusters_ = len(labels_unique)
        #
        #
        # print("number of estimated clusters : %d" % n_clusters_)
        #
        # plt.figure(1)
        # plt.clf()
        #
        # colors = cycle('bgrcmykbgrcmykbgrcmykbgrcmyk')
        # for k, col in zip(range(n_clusters_), colors):
        #     my_members = labels == k
        #     cluster_center = cluster_centers[k]
        #     plt.plot(X[my_members, 0], X[my_members, 1], col + '.')
        #     plt.plot(cluster_center[0], cluster_center[1], 'o', markerfacecolor=col,
        #              markeredgecolor='k', markersize=14)
        # plt.title('Estimated number of clusters: %d' % n_clusters_)
        # plt.show()

    def run(self):
        while not rospy.is_shutdown():
            self.sleepy.sleep()


person_detector = Person_Detection()
person_detector.run()
