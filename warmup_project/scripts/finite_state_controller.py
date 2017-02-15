#! /usr/bin/env python
"""
finite_state_controller_py
Kevin Zhang and Shane Kelly
CompRobo 2017

Implements an FSM structure to switch the Neato between two previously defined
states: driving in a square and person following.
"""

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

from visualization_msgs.msg import Marker


class FiniteStateController():
    """
    Contains all variables that define the shape/size of the square and the
    characteristics of the person following. Has a 'processScans' method that
    looks at LIDAR data for person following and an 'act' method that determines
    the state that the robot should be in and publishes 'twist_msg' motor commands
    based on the current state.
    """

    def __init__(self):
        #Init ROS things
        rospy.init_node('person_follow')
        self.mark = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.processScans, queue_size=10)

        #drive_square variables
        self.global_state = "drive_square"
        self.local_state = "forward"
        self.forward_dur = 5
        self.turn_dur = math.pi
        self.last_action_time = rospy.get_time()

        #person follow variables
        self.max_dist = 1.25
        self.current_error = None
        self.target_distance = rospy.get_param('~target_distance',.7)
        self.target_angle = 0
        self.k_lin = rospy.get_param('~k_lin', .75)
        self.k_ang = rospy.get_param('~k_ang', .5)
        self.raw_feet_scans = []
        self.filtered_feet_scans = []
        self.angles = range(30)
        self.angles.extend(range(330,360))
        self.centroid = None
        self.max_scans = 8
        self.past_centroid = (1, 0)

        #Marker stuff
        self.init_marker()

    def init_marker(self):
    """
    Creates a basic marker used to visualize the centroid of the person that
    the Neato is currently following.
    """

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
    """
    Runs every time a new set of LIDAR data is received. Creates an array of
    LIDAR points that have been determined to be the legs of a person to
    follow. This array is defined as the 10 points that are closest to centroid
    of the past 10 leg scans.
    """
        #This code all came from the person following module, it filters the laser scans for the top 8 shifted mean points
        self.raw_feet_scans = []
        self.filtered_feet_scans = []
        for i in self.angles:
            if msg.ranges[i] > 0.0 and msg.ranges[i] < self.max_dist:
                if i > 180:
                    self.raw_feet_scans.append((i-360, msg.ranges[i]))
                else:
                    self.raw_feet_scans.append((i, msg.ranges[i]))

        past_x = self.past_centroid[1] * math.cos(math.radians(self.past_centroid[0]))
        past_y = self.past_centroid[1] * math.sin(math.radians(self.past_centroid[0]))

        for point in self.raw_feet_scans:
            current_x = point[1] * math.cos(math.radians(point[0]))
            current_y = point[1] * math.sin(math.radians(point[0]))
            self.filtered_feet_scans.append((point[0], point[1], np.sqrt(np.square(current_x - past_x) + np.square(current_y - past_y))))

        self.filtered_feet_scans = sorted(self.filtered_feet_scans, key=lambda tup: tup[2])

        if len(self.filtered_feet_scans) > self.max_scans:
            self.filtered_feet_scans = self.filtered_feet_scans[:self.max_scans]

        if (len(self.filtered_feet_scans)):
            self.centroid = (np.mean([item[0] for item in self.filtered_feet_scans]), np.mean([item[1] for item in self.filtered_feet_scans]))
        else:
            self.centroid = (1, 0)
        self.past_centroid = self.centroid

        #Marker visualization
        point1 = self.centroid[1] * math.cos(math.radians(self.centroid[0]))
        point2 = self.centroid[1] * math.sin(math.radians(self.centroid[0]))

        self.marker.pose.position.x = -point1
        self.marker.pose.position.y = -point2
        self.marker.pose.position.z = 0
        self.mark.publish(self.marker)

    def act(self):
    """
    Selects the correct state based on the LIDAR input. Publishes 'twist_msg'
    motor commands based on the state of the Neato.
    """

        twist_msg = Twist()
        
        #While the nobody is front of the Neato, Neato will drive in a square
        if (self.global_state == "drive_square"):
            if (len(self.filtered_feet_scans) > 0):
                self.global_state = "person_follow"
            else:
                if (self.local_state == "forward"):
                    if (rospy.get_time() - self.last_action_time >= self.forward_dur):
                        twist_msg.angular.z = .5
                        self.local_state = "turn"
                        self.last_action_time = rospy.get_time()
                    else:
                        twist_msg.linear.x = .2
                elif (self.local_state == "turn"):
                    if (rospy.get_time() - self.last_action_time >= self.turn_dur):
                        twist_msg.linear.x = .2
                        self.local_state = "forward"
                        self.last_action_time = rospy.get_time()
                    else:
                        twist_msg.angular.z = .5
                        
        #While somebody is front of the Neato, start following them
        elif(self.global_state == "person_follow"):
            if (len(self.filtered_feet_scans) == 0):
                self.global_state = "drive_square"
            else:
                if self.centroid:
                    lin_error = self.centroid[1]*math.cos(math.radians(self.centroid[0])) - self.target_distance
                    ang_error = self.target_angle - self.centroid[0]
                    twist_msg.linear.x = self.k_lin * lin_error
                    twist_msg.angular.z = -self.k_ang * math.radians(ang_error)

        self.pub.publish(twist_msg)

if (__name__=="__main__"):
    """
    Main method, which initializes the FSM and runs it
    """
    
    fsm = FiniteStateController()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()

        fsm.act()
