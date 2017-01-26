#!/usr/bin/env python

from geometry_msgs.msg import Twist, Vector3
import rospy
import time
from inputs import get_gamepad

import threading

TURN_SPEED = 2
DRIVE_SPEED = 0.8

rospy.init_node('gamepad_teleop')
cmdPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

speed = Twist()


def publishStuff():
	r = rospy.Rate(20)
	while not rospy.is_shutdown():
		r.sleep()
		#time.sleep(0.1)

		print speed
		cmdPub.publish(speed)

threading.Thread(target=publishStuff).start()

r2 = rospy.Rate(1000)
while not rospy.is_shutdown():
	r2.sleep()
	events = get_gamepad()

	for event in events:
		if(event.code == 'ABS_X'):
			speed.angular.z = -(event.state - 128) / 128.0 * TURN_SPEED

		if(event.code == 'ABS_Y'):
			speed.linear.x = -(event.state - 128) / 128.0 * DRIVE_SPEED