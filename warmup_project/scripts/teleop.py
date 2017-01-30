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
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def keyToTwist(self, key):
        # global x
        # global ang
        # global lin_speed
        # global ang_speed

        if key == 'i':
            x = 1
            ang = 0
        elif key == ',':
            x = -1
            ang = 0
        elif key == 'j':
            x = 0
            ang = 1
        elif key == 'l':
            x = 0
            ang = -1
        elif key == 'u':
            x = 1
            ang = 1
        elif key == 'o':
            x = 1
            ang = -1
        elif key == 'm':
            x = -1
            ang = 1
        elif key == '.':
            x = -1
            ang = -1
        
        elif key == 'w':
            lin_speed *= 1.1
        elif key == 'x':
            lin_speed *= 0.9
        elif key == 'e':
            ang_speed *= 1.1
        elif key == 'c':
            ang_speed *= 0.9

        else:
            x = 0
            ang = 0

        twist = Twist()
        twist.linear.x = x*lin_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = ang*ang_speed
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
            while key != '\x03':
                key = getKey()
                twist = keyToTwist(key)
                pub.publish(twist)


        print "Teleop is finished."


    # settings = termios.tcgetattr(sys.stdin)
    # key = None
    # lin_speed = 0.5
    # ang_speed = 1.0

    # x = 0
    # y = 0
    # z = 0
    # ang = 0

if __name__ == '__main__':
    node = Teleop()