#! /usr/bin/env python
"""
wall_follower.py
Shane Kelly and Kevin Zhang
CompRobo 2017

This script allows the Neato to find and follow a wall from a set distance away (set a 1m).
The script uses dual proportional control to find the nearest wall and then drive parallel to it.
"""

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker


class Wall_Follower():
    """
    The object which contains the ROS nodes and the methods that define wall-following functionality
    Holds the act() method, which makes the Neato move based on the readings from the processScans()
    """

    def __init__(self):
        #Init ROS things
        rospy.init_node('wall_follow')
        self.mark = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.processScans, queue_size=10)

        #Arguments for proportional control
        self.target_distance = rospy.get_param('~target_distance',1)
        self.k = rospy.get_param('~k', .5)
        self.current_error = None


        #The two readings for proportional control
        self.front_reading = 0
        self.back_reading = 0

        #Initializes wall marker
        self.init_marker()


    def init_marker(self):
        """
        Creates a marker that visualizes the wall which the Neato is trying to follow
        """

        self.marker = Marker()
        self.marker.type = 4
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
        Callback for LIDAR data from /scan
        Takes in msg, which is a LaserScan data type that holds a variable called ranges that holds 361 datapoints,
        which correspond to the laser scans from degree 0-360 inclusive

        Takes in the scans at 240 and 300 degrees, and stores them in variables
        """

        self.front_reading = msg.ranges[300]
        self.back_reading = msg.ranges[240]
        #Checks for bad readings
        if self.front_reading != 0 and self.back_reading != 0:
            self.current_error = self.front_reading - self.back_reading


    def act(self):
        """
        Calculates the next movement of the Neato in Twist msg using proportional control
        The objective is to try and minimize the difference between the two readings to 0

        Publishes to /cmd_vel the Twist msg which contains linear and angular velocity for the Neato to move
        """

        twist_msg = Twist()

        #If the readings are bad, then the Neato just moves forward slowly for this current iteration
        if self.front_reading == 0 or self.back_reading == 0:
            twist_msg.linear.x = .05
        elif self.current_error:
            point1 = Point(x=-math.cos(math.radians(300))*self.front_reading, y=-math.sin(math.radians(300))*self.front_reading)
            point2 = Point(x=-math.cos(math.radians(240))*self.back_reading, y=-math.sin(math.radians(240))*self.back_reading)

            #Marker Visualization
            self.marker.points = [point1, point2]
            self.mark.publish(self.marker)

            #The main thing that changes is the angle at which the Neato moves, which determines how much it turns
            error = self.front_reading - self.target_distance
            twist_msg.linear.x = .2
            twist_msg.angular.z += -self.current_error*self.k - error*self.k

        self.pub.publish(twist_msg)




if (__name__=="__main__"):
    """
    Main method, which initializes and runs the Neato on Wall_follower()
    """

    wallee = Wall_Follower()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        wallee.act()
