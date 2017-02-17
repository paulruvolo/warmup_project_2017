#! /usr/bin/env python
"""
teleop.py
Kevin Zhang and Shane Kelly
CompRobo Spring 2017

This script allows for manual control of the Neato using keypresses.
"""

import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist

class Teleop():
    """
    The object which holds key input functionality and determines what actions the Neato should take based on key press
    """

    def __init__(self):
        #Init ROS things
        rospy.init_node('teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #Keeps track of keys and outputs
        self.key = None
        self.lin_vel = .3
        self.ang_vel = 1

    def getKey(self):
        """
        Uses the sys library to read key inputs
        Stores the value of the key presses in self.key
        """

        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        self.key =  key

    def act(self):
        """
        Determines the next movement of the Neato in Twist msg based on what key was pressed
        A conditional tree is used to separate different keys and commands
        Commands include moving in the 4 cardinal directions and the 4 intermediate directions,
        as well as changing speed

        Publishes to /cmd_vel a Twist msg that holds the linear and angular velocities for the Neato
        """
        twist_msg = Twist()
        if (self.key == 'w'):
            twist_msg.linear.x = self.lin_vel
        elif (self.key == 'a'):
            twist_msg.angular.z = self.ang_vel
        elif (self.key == 's'):
            twist_msg.linear.x = 0
        elif (self.key == 'd'):
            twist_msg.angular.z = -self.ang_vel
        elif (self.key == 'q'):
            twist_msg.linear.x = self.lin_vel
            twist_msg.angular.z = self.ang_vel
        elif (self.key == 'e'):
            twist_msg.linear.x = self.lin_vel
            twist_msg.angular.z = -self.ang_vel
        elif (self.key == 'x'):
            twist_msg.linear.x = -self.lin_vel
        elif (self.key == 'z'):
            twist_msg.linear.x = -self.lin_vel
            twist_msg.angular.z = -self.ang_vel
        elif (self.key == 'c'):
            twist_msg.linear.x = -self.lin_vel
            twist_msg.angular.z = self.ang_vel
        elif (self.key == 'h'):
            self.lin_vel -= .05
            print "lin_vel:", self.lin_vel
        elif (self.key == 'j'):
            self.lin_vel += .05
            print "lin_vel:", self.lin_vel
        elif (self.key == 'n'):
            self.ang_vel -= .1
        elif (self.key == 'm'):
            self.ang_vel += .1

        self.pub.publish(twist_msg)

if (__name__=="__main__"):
    """
    Main method which initializes Teleop and waits for keys to send commands to the Neato
    """

    settings = termios.tcgetattr(sys.stdin)
    teleop = Teleop()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown() and teleop.key != '\x03'):
        r.sleep()
        teleop.getKey()
        teleop.act()
