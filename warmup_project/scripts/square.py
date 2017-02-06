#!/usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
import rospy
import math
from time import time


import tty
import select
import sys
import termios

class Control_Robot():

    def __init__(self):
        """ Initialize the robot control, """
        rospy.init_node('square')
        rospy.Subscriber('/odom', Odometry, self.processOdom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        self.sleepy = rospy.Rate(2)

        # make dictionary that calls functions
        self.state = {'i':self.forward, ',':self.backward,
                      'l':self.rightTurn, 'j':self.leftTurn,
                      'k':self.stop}

        self.x = 0 # position in meters
        self.y = 0 # position in meters
        self.z = 0 # angle in degrees
        self.desiredX = 0
        self.desiredY = 0
        self.desiredZ = 0

        self.linearVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.sendMessage()

        self.start = time()

        # get key interupt things
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None


    def getKey(self):
        """ Interupt (I think) that get a non interrupting keypress """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


    def forward(self):
        """
            Sets the velocity to forward
            Only checking for bump sensors in forward
        """
        print('forward')
        self.linearVector  = Vector3(x=1.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)


    def backward(self):
        print('backward')
        """ Sets the velocity to backward """
        self.linearVector  = Vector3(x=-1.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)


    def leftTurn(self):
        print('leftTurn')
        """ Sets the velocity to turn left """
        self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=1.0)


    def rightTurn(self):
        print('rightTurn')
        """ Sets the velocity to turn right """
        self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=-1.0)


    def stop(self):
        """ Sets the velocity to stop """
        print('stop')
        self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)


    def sendMessage(self):
        """ Publishes the Twist containing the linear and angular vector """
        print('sendMessage')
        self.pub.publish(Twist(linear=self.linearVector, angular=self.angularVector))


    def processOdom(self, msg):
        """ processes the odom into x, y, and angle z """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = 180 * (msg.pose.pose.orientation.z % 2)
        print(self.z)


    def turn90(self):
        self.desiredZ = (self.z + 90) % 360
        while self.desiredZ > self.z:
            print('desiredZ: ', self.desiredZ)
            print('z: ', self.z)
            self.leftTurn()
            self.sendMessage()

    def move1m(self):
        self.desiredX = self.x + math.cos(self.z)
        self.desiredY = self.y + math.sin(self.z)
        while math.sqrt((self.desiredX - self.x)**2 + (self.desiredY - self.y)**2) < .1:
            print(math.sqrt((self.desiredX - self.x)**2 + (self.desiredY - self.y)**2))
            self.forward()
            self.sendMessage()

    def run(self):
        while (self.key != '\x03') and (not rospy.is_shutdown()):
            self.getKey()
            try:
                self.state[self.key].__call__()
            except:
                # on any other keypress, stop the robot
                self.state['k'].__call__()


            self.sendMessage()
            self.sleepy.sleep()


    def run2(self):
        while (not rospy.is_shutdown()) and (self.start + 2.0 > time()):
            self.turn90()
            self.move1m()
            self.sleepy.sleep()


control = Control_Robot()
control.run2()
# control.run()
