#!/usr/bin/env python

from geometry_msgs.msg import Twist

import tty
import select
import sys
import termios
import rospy

class Teleop(object):
    def __init__(self):
        rospy.init_node('teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.key = None
        self.lin_speed = 0.5
        self.ang_speed = 1.0
        self.x = 0
        self.y = 0
        self.z = 0
        self.ang = 0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def keyToTwist(self):
        key_bindings = { # key to x, ang values
            'i': (1, 0),
            ',': (-1, 0),
            'j': (0, 1),
            'l': (0, -1),
            'u': (1, 1),
            'o': (1, -1),
            'm': (-1, 1),
            '.': (-1, -1)
        }

        if self.key in key_bindings:
            self.x = key_bindings[self.key][0]
            self.ang = key_bindings[self.key][1]       
        elif self.key == 'w':
            self.lin_speed *= 1.1
        elif self.key == 'x':
            self.lin_speed *= 0.9
        elif self.key == 'e':
            self.ang_speed *= 1.1
        elif self.key == 'c':
            self.ang_speed *= 0.9
        else:
            self.x = 0
            self.ang = 0

        twist = Twist()
        twist.linear.x = self.x*self.lin_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.ang*self.ang_speed
        return twist

    def drive(self):
        keypad = """

        To move around:

        u   i   o
        j   k   l
        m   ,   .

        w/x: increase/decrease linear speed by 10%
        e/c: increase/decrease angular speed by 10%

        """

        print keypad
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            while self.key != '\x03':
                self.getKey()
                twist = self.keyToTwist()
                self.pub.publish(twist)


        print "Teleop is finished."

if __name__ == '__main__':
    node = Teleop()
    node.drive()