#! /usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

class Person_Follower():

    def __init__(self):
        rospy.init_node('person_follow')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.processScans, queue_size=10)
        self.current_error = None
        self.target_distance = rospy.get_param('~target_distance',1)
        self.target_angle = 0
        self.k = rospy.get_param('~k', .5)
        self.feet_scans = []
        self.angles = range(15)
        self.angles.extend(range(345,360))
        self.centroid = None



    def processScans(self, msg):
        for i in self.angles:
            if msg.ranges[i] < 3:
                self.feet_scans.append((i, msg.ranges[i]))
        self.centroid = (float(sum([item(0) for item in self.feet_scans]))/len(self.feet_scans), float(sum([item(1) for item in self.feet_scans]))/len(self.feet_scans))


    def act(self):

        #TODO
        
        twist_msg = Twist()


        if self.front_reading == 0 or self.back_reading == 0:
            twist_msg.linear.x = .05
        elif self.current_error:
            point1 = Point(x=-math.cos(math.radians(300))*self.front_reading, y=-math.sin(math.radians(300))*self.front_reading)
            point2 = Point(x=-math.cos(math.radians(240))*self.back_reading, y=-math.sin(math.radians(240))*self.back_reading)
            self.marker.points = [point1, point2]
            self.mark.publish(self.marker)

            error = self.front_reading - self.target_distance
            twist_msg.linear.x = .2
            twist_msg.angular.z += -self.current_error*self.k - error*self.k

        self.pub.publish(twist_msg)




if (__name__=="__main__"):
    peoples = Person_Follower()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        peoples.act()
