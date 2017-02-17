#!/usr/bin/env python

'''
    This node detects a wall using a laser scan
    and publishes a line marker that represents
    the wall
    TODO: Detect walls on both sides of the neato
'''

import rospy, math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose, Point
from std_msgs.msg import Header, ColorRGBA

rospy.init_node('wall_detect')

class WallDetect(object):
    def __init__(self):
        '''
            Initializes local variables and the publisher and subscriber
        '''
        self.nthile = 0.1
        self.pub = rospy.Publisher('detect', Marker, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.laser_to_cart)

    def laser_to_cart(self, scan):
        '''
            Converts a laser scan message to cartesian and calls
            publish line with the scan header and two points from
            the nthile range
            scan: a laser scan message
        '''
        laser_points = []
        angle = scan.angle_min
        for point in scan.ranges:
            #convert to cartesian in here
            x = point*math.cos(angle)
            y = point*math.sin(angle)
            angle += scan.angle_increment
            if(y > 0 and abs(x) < 1.5 and point != 0):
                laser_points.append(Point(x, y, 0.0))

        index = int(len(laser_points) * self.nthile)
        print len(laser_points)
        if len(laser_points) >= 2:
            self.publish_line(scan.header, [laser_points[index], laser_points[-index-1]])

    def publish_line(self, header, line_points):
        '''
            Publishes a marker with the given
            header and array of points
            header: a ros header
            line_points: an array of points
        '''
        marker_message = Marker(id=0, type=Marker.LINE_STRIP,
            header=header,
            pose=Pose(position=Point(0.0,0.0,0.0)), scale=Vector3(0.1,0.1,0.1),
            color = ColorRGBA(g=1,a=1), points=line_points)
        self.pub.publish(marker_message)

    def run(self):
        '''
            Runs the wall detector
        '''
        rospy.spin()

if __name__ == '__main__':
    WallDetect().run()