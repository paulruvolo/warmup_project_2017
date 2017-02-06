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


def go_to_theta():
    """Given an x,y point , goes to point"""
    velcmd = Twist(Vector3(0,0,0),Vector3(0,0,1))
    pub.publish(velcmd)
    time.sleep(1.65)
    velcmd = Twist(Vector3(0,0,0),Vector3(0,0,0))

def drive_1_m():
    velcmd = Twist(Vector3(1,0,0),Vector3(0,0,0))
    pub.publish(velcmd)
    time.sleep(3.68)
    velcmd = Twist(Vector3(1,0,0),Vector3(0,0,0))

  


#start node
rospy.init_node('arpan_squaretime')
#publisher to publish to /vmd_vel
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
r = rospy.Rate(2)

time.sleep(1)

while not rospy.is_shutdown():
    for i in range(1,3,1):
        go_to_theta()
        drive_1_m()
        r.sleep()
