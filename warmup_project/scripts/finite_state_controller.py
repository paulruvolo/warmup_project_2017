#!usr/bin/env python

from person_follower import FollowPerson
from wall_follower import FollowWall
from geometry_msgs.msg import Twist
import rospy

if __name__ == '__main__':
	rospy.init_node('finite_state')
	person_follower = FollowPerson()
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	r = rospy.Rate(5)
	while not rospy.is_shutdown():
		pf_twist = person_follower.run_finite()
		if not person_follower.person_present:
			twist = Twist()
			twist.angular.z = 0.6
			pub.publish(twist)
		else:
			pub.publish(pf_twist)
			
		r.sleep()