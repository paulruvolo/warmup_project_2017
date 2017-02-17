#!/usr/bin/env python
"""
Drive the robot in a square
"""

from geometry_msgs.msg import Twist, Vector3
import rospy
import time
from math import pi

rospy.init_node('square_controller')
cmdPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Allow the node to initialize before publishing data
time.sleep(.5)

# TODO: these should be ROS params
TURN_SPEED = 1  # rad/sec
DRIVE_SPEED = 0.8  # m/s
SIDE_LENGTH = 1  # meters

def driveSquare():
	for _ in xrange(4):
		driveSide()

	# Stop the robot
	cmdPub.publish(Twist())

def driveSide():
	# Drive one side of the square, then turn.
	cmdPub.publish(Twist(linear=Vector3(x=DRIVE_SPEED)))
	time.sleep(SIDE_LENGTH / DRIVE_SPEED)
	cmdPub.publish(Twist(angular=Vector3(z=TURN_SPEED)))
	time.sleep((pi/2) / TURN_SPEED)

if __name__ == '__main__':
	driveSquare()