#! /usr/bin/env python
"""
drive_square.py
Kevin Zhang and Shane Kelly
CompRobo 2017

Controls a Neato to drive in a perfect square with side lengths of 1m.
"""

import rospy
from geometry_msgs.msg import Twist

class Square():
    """
    Holds attributes that define the shape and size of the driven square. Also
    contains an 'act' method that gives the robot commands to drive in the
    desired square.
    """

    def __init__(self):
        #Init ROS things
        rospy.init_node('square')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        #Square variables
        self.last_action_time = rospy.get_time()
        self.state = "forward"
        self.forward_dur = 2
        self.turn_dur = 6.28

    def act(self):
        """
        Publishes 'twist_msg' motion commands to drive in the desired square. 
        """

        twist_msg = Twist()
        
        #State forward - go forward for 2 seconds
        if (self.state == "forward"):
            if (rospy.get_time() - self.last_action_time >= self.forward_dur):
                twist_msg.angular.z = .25
                self.state = "turn"
                self.last_action_time = rospy.get_time()
            else:
                twist_msg.linear.x = .2
                
        #State turn - turn left for 6.28 seconds
        elif (self.state == "turn"):
            if (rospy.get_time() - self.last_action_time >= self.turn_dur):
                twist_msg.linear.x = .2
                self.state = "forward"
                self.last_action_time = rospy.get_time()
            else:
                twist_msg.angular.z = .25
        self.pub.publish(twist_msg)



if (__name__=="__main__"):
    """
    Main method, which initializes Square and runs it
    """
    
    square = Square()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        square.act()
