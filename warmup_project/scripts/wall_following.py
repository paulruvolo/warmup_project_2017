#!/usr/bin/env python

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy
import math

class Wall_Follower():

    def __init__(self):
        rospy.init_node('wall_follower')
        rospy.Subscriber('/scan', LaserScan, self.laserCallback)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.sleepy = rospy.Rate(2)

        self.linearVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)

        self.angleError = 0
        self.P = 40.0
        self.offset = 2.5

        self.sendMessage()



    def laserCallback(self, msg):
        print("laserCallback")

        leftFrontQuadrant = msg.ranges[0:90]
        leftBackQuadrant  = msg.ranges[90:180]
        rightBackQuadrant   = msg.ranges[180:270]
        rightFrontQuadrant  = msg.ranges[270:360]

        diff = 0

        for rf, rb in zip(rightFrontQuadrant, reversed(rightBackQuadrant)):
            # print ("right side")
            # print (rf, rb)
            if rf == 0.0 or rb == 0.0:
                continue
            diff += rb - rf

        for lf, lb in zip(leftFrontQuadrant, reversed(leftBackQuadrant)):
            # print("left side")
            # print (lf, lb)
            if lf == 0.0 or lb == 0.0:
                continue
            diff += lf - lb

        self.angleError = (math.tanh((diff-self.offset)/self.P))
        print(self.angleError)

    def correctAngle(self):
        """ Uses P control to corrtect the angle of the neato """

        self.linearVector = Vector3(x=0.2, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=self.angleError)


    def sendMessage(self):
        """ Publishes the Twist containing the linear and angular vector """
        print("sendMessage")
        self.pub.publish(Twist(linear=self.linearVector, angular=self.angularVector))


    def run(self):
        while not rospy.is_shutdown():
            self.correctAngle()
            self.sendMessage()
            self.sleepy.sleep()


wall_follower = Wall_Follower()
wall_follower.run()
