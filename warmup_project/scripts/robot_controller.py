#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from geometry_msgs.msg import PoseWithCovariance,TwistWithCovariance,Twist,Vector3,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import select
import sys
import termios
import math
import time
import tty
import select
import sys
import termios

class Control_Robot():

    def __init__(self):
        """ Initialize the robot control, """
        rospy.init_node('robot_controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sleepy = rospy.Rate(2)
        rospy.Subscriber("/odom",Odometry,self.process_odom)
        rospy.Subscriber("/person_point",Point,self.process_person)
        rospy.Subscriber("/clear_path_point",Point,self.process_clear_path)

        rospy.on_shutdown(self.stop)

        # make dictionary that calls functions for teleop
        self.state = {'i':self.forward, ',':self.backward,
                      'l':self.rightTurn, 'j':self.leftTurn,
                      'k':self.stop,'n':self.personfollowing,
                      'b':self.clearPathFollowing,'v':self.multistate}
        self.acceptablekeys = ['i','l','k',',','j','n','b','v']
        self.linearVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.sendMessage()
        # get key interupt things
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        #current location and orientation
        self.currentx = 0.0
        self.currenty = 0.0
        self.orientation = 0.0
        #proportional controller constants
        self.kturn = .85
        self.kspeed= .1
        #location of person to be followed
        self.personx = 0.0
        self.persony = 0.0
        #location of target for obstacle avoidance
        self.clearx = 0.0
        self.cleary = 0.0

    def getKey(self):
        """ Interupt that gets a non interrupting keypress """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def forward(self):
        """
            Sets the velocity to forward onkeypress
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
        #self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        #self.angularVector = Vector3(x=0.0, y=0.0, z=-1.0)
        self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=-1.0)

    def personfollowing(self):
        """Runs personfollowing"""
        print('personfollowing')
        while not rospy.is_shutdown():
                print self.personx
                print self.persony
                self.goto_point(self.personx,self.persony)
                self.sendMessage()

    def clearPathFollowing(self):
        """Runs personfollowing"""
        print('personfollowing')
        while not rospy.is_shutdown():
                print self.clearx
                print self.cleary
                self.goto_point(self.clearx,self.cleary)
                self.sendMessage()

    def multistate(self):
        """Personfollows if person is further than half a meter away, otherwise avoids obstacles"""
        while not rospy.is_shutdown():
            persondistance = math.sqrt((self.currentx-personx)**2 + (self.currenty-persony)**2)
            if persondistance <= .5:
                self.goto_point(self.clearx,self.cleary)
                print 'avoiding'
            else:
                self.goto_point(self.personx,self.persony)
                print 'following'
            self.sendMessage()

    def stop(self):
        """ Sets the velocity to stop """
        print('stop')
        self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.sendMessage()
        print "currentx = " + str(self.currentx)
        print "currenty = " + str(self.currenty)
        print "orientation = " + str(self.orientation)

    def sendMessage(self):
        """ Publishes the Twist containing the linear and angular vector """
        print('sendMessage')
        self.pub.publish(Twist(linear=self.linearVector, angular=self.angularVector))

    def process_person(self,msg):
        """Starts personfollowing on recieved person"""
        self.personx = msg.x
        self.persony = msg.y

    def process_clear_path(self,msg):
        """Starts personfollowing on recieved person"""
        self.clearx = msg.x
        self.cleary = msg.y

    def process_odom(self,msg):
        orientation_tuple = (msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        self.currentx = msg.pose.pose.position.x
        self.currenty = msg.pose.pose.position.y
        self.orientation = angles[2]

    def angle_normalize(self,z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(self,a,b):
        """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
        """
        self.a = self.angle_normalize(a)
        self.b = self.angle_normalize(b)
        self.d1 = a-b
        self.d2 = 2*math.pi - math.fabs(self.d1)
        if self.d1 > 0:
            self.d2 *= -1.0
        if math.fabs(self.d1) < math.fabs(self.d2):
            return self.d1
        else:
            return self.d2

    def goto_point(self,targetx,targety):
        """Drives to specified point, proportionally controlling turnrate and speed"""
        #if point is 0,0, make 0.01,0.01 to avoid divide by 0
        if targetx == 0 and targety == 0:
            targetx = 0.01
            targety = 0.01
        self.targetdistance = math.sqrt((self.currentx-targetx)**2 + (self.currenty-targety)**2)
        self.targetangle = math.atan2(targety-self.currenty,targetx-self.currentx)
        self.angledifference  = self.angle_diff(self.targetangle,self.orientation)
        if abs(self.angledifference) < .10:
            self.turnspeed = 0
        else:
            self.turnspeed = math.tanh(self.kturn*self.angledifference)
        self.speed = math.tanh(self.targetdistance*self.kspeed/self.angledifference)
        if self.speed < 0:
            self.speed = 0
        self.linearVector = Vector3(x=self.speed, y=0.0, z=0.0)
        self.angularVector = Vector3(x = 0.0, y = 0.0, z = self.turnspeed)
        # print "currentx = " + str(self.currentx)
        # print "currenty = " + str(self.currenty)
        # print "orientation = " + str(self.orientation)
        # print "targetangle = " + str(self.targetangle)
        # print "angledifference = " + str(self.angledifference)
        print "turnspeed = " + str(self.turnspeed)
        print "speed = " + str(self.speed)

    def run(self):
        
        while self.key != '\x03' and not rospy.is_shutdown():
            self.getKey()
            if self.key in self.acceptablekeys:
                #if an acceptable keypress, do the action
                self.state[self.key].__call__()
            else:
                # on any other keypress, stop the robot
                self.state['k'].__call__()
            self.sendMessage()

control = Control_Robot()
control.run()
