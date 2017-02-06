#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

rospy.init_node('rate_test')

def onLaserScan(scan):
    print "Laser scan age: {}sec".format((rospy.Time.now() - scan.header.stamp).to_sec())

laserSub = rospy.Subscriber('stable_scan', LaserScan, onLaserScan)

rospy.spin()
