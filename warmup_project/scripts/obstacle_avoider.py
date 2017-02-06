#! /usr/bin/env python

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker

class ObstacleAvoidance():

    def __init__(self):
        rospy.init_node('obstacle_avoid')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, \
                                    self.processScans, queue_size=10)

        self.obstacles = []
        self.num_left_obst = 0
        self.num_right_obst = 0
        self.max_dist = .75
        self.last_turn = None
        self.last_time = rospy.get_time()
        self.state = "forward"
        self.sideways = False
        self.ang_off = 0
        self.count = 0

    def init_marker(self):
        self.marker = Marker()
        self.marker.type = 2
        self.marker.header.frame_id = "base_laser_link"
        self.marker.scale.x = .1
        self.marker.scale.y = .1
        self.marker.scale.z = .1
        self.marker.color.a = 1
        self.marker.color.r = 0
        self.marker.color.g = 1
        self.marker.color.b = 1

    def processScans(self, msg):
        if (1==1):
            self.obstacles = []
            self.num_left_obst = 0
            self.num_right_obst = 0

            for i in range(len(msg.ranges)):
                angle = i
                distance = msg.ranges[i]
                if (distance > 0.0 and distance < self.max_dist):
                    upper = self.ang_off+20
                    lower = self.ang_off-20
                    if (lower < 0):
                        lower += 360
                    if (lower == 340):
                        if (angle > lower or angle < upper):
                            self.obstacles.append((angle, distance))
                    else:
                        if (angle > lower and angle < upper):
                            self.obstacles.append((angle, distance))
            for obst in self.obstacles:
                if (obst[0] < upper):
                    self.num_left_obst += 1
                elif (obst[0] > lower):
                    self.num_right_obst += 1

    def act(self):
        twist_msg = Twist()
        if (self.state == "forward"):
            if (not self.sideways):
                if (len(self.obstacles) > 0):
                    self.count += 1
                    if (self.count > 2):
                        self.count = 0
                        if (self.num_left_obst >= self.num_right_obst):
                            twist_msg.angular.z = -.5
                            self.state = "right turn"
                            self.last_turn = "right"
                            self.last_time = rospy.get_time()
                            self.ang_off += 90
                            if (self.ang_off >= 360):
                                self.ang_off -= 360
                        else:
                            twist_msg.angular.z = .5
                            self.state = "left turn"
                            self.last_turn = "left"
                            self.last_time = rospy.get_time()
                            self.ang_off -= 90
                            if (self.ang_off < 0):
                                self.ang_off += 360
                else:
                    twist_msg.linear.x = .1
            else:
                if (len(self.obstacles) == 0):
                    if (self.last_turn == "left"):
                        twist_msg.angular.z = -.5
                        self.state = "right turn"
                        self.last_turn = "right"
                        self.last_time = rospy.get_time()
                        self.ang_off += 90
                        if (self.ang_off >= 360):
                            self.ang_off -= 360
                    else:
                        twist_msg.angular.z = .5
                        self.state = "left turn"
                        self.last_turn = "left"
                        self.last_time = rospy.get_time()
                        self.ang_off -= 90
                        if (self.ang_off < 0):
                            self.ang_off += 360
                else:
                    twist_msg.linear.x = .1
        elif (self.state == "left turn"):
            if (rospy.get_time() - self.last_time >= math.pi):
                twist_msg.linear.x = .1
                self.state = "forward"
                self.sideways = not self.sideways
            else:
                twist_msg.angular.z = .5
        elif (self.state == "right turn"):
            if (rospy.get_time() - self.last_time >= math.pi):
                twist_msg.linear.x = .1
                self.state = "forward"
                self.sideways = not self.sideways
            else:
                twist_msg.angular.z = -.5

        self.pub.publish(twist_msg)

if (__name__=="__main__"):
    obst_avoid = ObstacleAvoidance()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        obst_avoid.act()
