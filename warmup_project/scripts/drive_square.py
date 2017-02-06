#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class Square():

    def __init__(self):
        rospy.init_node('square')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.last_action_time = rospy.get_time()
        self.state = "forward"
        self.forward_dur = 2
        self.turn_dur = 6.28

    def act(self):
        twist_msg = Twist()
        if (self.state == "forward"):
            if (rospy.get_time() - self.last_action_time >= self.forward_dur):
                twist_msg.angular.z = .25
                self.state = "turn"
                self.last_action_time = rospy.get_time()
            else:
                twist_msg.linear.x = .2
        elif (self.state == "turn"):
            if (rospy.get_time() - self.last_action_time >= self.turn_dur):
                twist_msg.linear.x = .2
                self.state = "forward"
                self.last_action_time = rospy.get_time()
            else:
                twist_msg.angular.z = .25
        self.pub.publish(twist_msg)



if (__name__=="__main__"):
    square = Square()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        square.act()
