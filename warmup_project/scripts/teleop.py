#!/usr/bin/env python

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
import rospy

import tty
import select
import sys
import termios

class Control_Robot():

    def __init__(self):
        """ Initialize the robot control, """
        rospy.init_node('my_teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sleepy = rospy.Rate(2)

        # make dictionary that calls functions
        self.state = {'i':self.forward, ',':self.backward,
                      'l':self.rightTurn, 'j':self.leftTurn,
                      'k':self.stop}

        self.linearVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.sendMessage()

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


    def run(self):
        while self.key != '\x03':
            self.getKey()
            try:
                self.state[self.key].__call__()
            except:
                # on any other keypress, stop the robot
                self.state['k'].__call__()
            self.sendMessage()

control = Control_Robot()
control.run()
