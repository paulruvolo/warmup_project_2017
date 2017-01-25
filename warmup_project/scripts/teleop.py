#! /usr/bin/env python

import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist

class Teleop():

    def __init__(self):
        rospy.init_node('marker')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.key = None
        self.lin_vel = .3
        self.ang_vel = 1

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        self.key =  key

    def act(self):
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
    settings = termios.tcgetattr(sys.stdin)    
    teleop = Teleop()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown() and teleop.key != '\x03'):
        r.sleep()
        teleop.getKey()
        teleop.act()
