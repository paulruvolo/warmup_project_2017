#!/usr/bin/env python

#This node drives in a square on I and stops the robot on any other keypress
import rospy
from geometry_msgs.msg import PoseWithCovariance,TwistWithCovariance,Twist,Vector3
from nav_msgs.msg import Odometry
import select
import sys
import termios
import math
import time

currentx = 0.0
currenty = 0.0
orientation = 0.0

def process_odom(msg):
    """Processes a recieved odom msg"""
    global currentx
    global currenty
    global orientation
    currentx = msg.pose.pose.position.x 
    currenty = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation.z


def go_to_theta(desiredtheta):
    """Given an x,y point , goes to point"""
    dtheta = orientation - desiredtheta
    thetarate = dtheta*2
    if thetarate < -1:
        thetarate = -1
    if thetarate > 1:
        thetarate = 1
    if abs(dtheta) < .05:
        velcmd = Twist(Vector3(0,0,0),Vector3(0,0,0))
        return "Done"
    print ('dtheta:',dtheta)
    print ('orientation:',orientation)
    velcmd = Twist(Vector3(0,0,0),Vector3(0,0,thetarate))
    pub.publish(velcmd)

def drive_1_m(startx,starty):
    dist = math.sqrt((startx-currentx)**2 + (starty-currenty)**2)
    if dist > .95:
        return "Done"
    deltav = (1-dist) * .25
    print ('distance: ', dist)
    velcmd = Twist(Vector3(deltav,0,0),Vector3(0,0,0))
    pub.publish(velcmd)


#start node
rospy.init_node('arpan_driver')
#publisher to publish to /vmd_vel
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
rospy.Subscriber("/odom",Odometry,process_odom)
r = rospy.Rate(2)

time.sleep(1)
point1 = (currentx,currenty+1)
point2 = (currentx+1,currenty+1)
point3 = (currentx+1,currenty)

while not rospy.is_shutdown():
        desiredtheta = orientation +.5
        if desiredtheta < -1:
            desiredtheta += 2
        if desiredtheta > 1:
            desiredtheta -= 2
        while go_to_theta(desiredtheta) != "Done":
            go_to_theta(desiredtheta)
        startx = currentx
        starty = currenty
        while drive_1_m(startx,starty) != "Done":
            drive_1_m(startx,starty)
            r.sleep()


