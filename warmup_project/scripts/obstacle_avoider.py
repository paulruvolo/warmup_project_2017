#! /usr/bin/env python
"""
obstacle_avoider.py
Kevin Zhang and Shane Kelly
CompRobo 2017

Makes the Neato attempt to move in a single direction. If anything obstructs
the Neato's path, it will move sideways until the obstruction is gone and then
resume heading toward its goal.
"""

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker

class ObstacleAvoidance():
    """
    Holds characteristics that define the obstacle avoidance behavior. Has a
    callback 'processScans' for the LIDAR data that looks to find obstacles and
    an 'act' method that publishes 'twist_msg' motor commands based on the
    current state of the robot and current LIDAR data.
    """

    def __init__(self):
        #Init ROS things
        rospy.init_node('obstacle_avoid')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, \
                                    self.processScans, queue_size=10)
        #Obstacle variables
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

    def processScans(self, msg):
        """
        Analyzes relevant LIDAR data, finds obstacles that are obstructing the
        Neato and sorts them into left obstacles and right obstacles for later
        interpretation.
        """

        if (1==1):
            #Resets the obstacle variables
            self.obstacles = []
            self.num_left_obst = 0
            self.num_right_obst = 0

            #Calculates whether obstacles exist in front of Neato
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
            #Classify obstacles into either left of right
            for obst in self.obstacles:
                if (obst[0] < upper):
                    self.num_left_obst += 1
                elif (obst[0] > lower):
                    self.num_right_obst += 1

    def act(self):
        """
        Publishes 'twist_msg' motor commands to make the Neato avoid obstacles
        based on the LIDAR input data and the current state of the Neato. Structure
        resembles an FSM.
        """
        
        twist_msg = Twist()
        
        #State forward - move forward until you encounter an obstacle
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
                    
        #State left turn - if more of the obstacle is to the right of the Neato, turn left
        elif (self.state == "left turn"):
            if (rospy.get_time() - self.last_time >= math.pi):
                twist_msg.linear.x = .1
                self.state = "forward"
                self.sideways = not self.sideways
            else:
                twist_msg.angular.z = .5
                
        #State right turn - if more of the obstaacle is to the left of the Neato, turn right
        elif (self.state == "right turn"):
            if (rospy.get_time() - self.last_time >= math.pi):
                twist_msg.linear.x = .1
                self.state = "forward"
                self.sideways = not self.sideways
            else:
                twist_msg.angular.z = -.5

        self.pub.publish(twist_msg)

if (__name__=="__main__"):
    """
    Main method, which initializes the ObstacleAvoidance class and runs it
    """
    
    obst_avoid = ObstacleAvoidance()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        obst_avoid.act()
