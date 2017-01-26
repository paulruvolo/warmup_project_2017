#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Wall_Follower():

    def __init__(self):
        rospy.init_node('wall_follow')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.processScans, queue_size=10)
        self.current_error = None
        self.target_distance = rospy.get_param('~target_distance',1)
        self.k = rospy.get_param('~k', .5)
        self.front_reading = 0
        self.back_reading = 0

    def processScans(self, msg):
        self.front_reading = msg.ranges[300]
        self.back_reading = msg.ranges[240]
        if self.front_reading != 0 and self.back_reading != 0:
            self.current_error = self.front_reading - self.back_reading

    def act(self):
        twist_msg = Twist()
        if self.front_reading == 0 or self.back_reading == 0:
            pass
        elif self.current_error:

            error = self.front_reading - self.target_distance
            twist_msg.linear.x = .2
            twist_msg.angular.z += -self.current_error*self.k - error*self.k

        self.pub.publish(twist_msg)



if (__name__=="__main__"):
    wallee = Wall_Follower()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        wallee.act()
