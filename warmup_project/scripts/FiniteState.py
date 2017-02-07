#! /usr/bin/env python

import rospy

rospy.init_node('finite_state')

class FiniteState(object):
	def __init__(self):
		super(FiniteState, self).__init__()