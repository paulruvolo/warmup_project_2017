#!/usr/bin/env python

import rospy
import tf
import math

from sensor_msgs.msg import LaserScan, PointCloud
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose, Point, PointStamped
from std_msgs.msg import ColorRGBA, Header


def laser_to_pointcloud(scan):
    """
    :type scan: LaserScan
    """
    points = []
    angle = scan.angle_min
    for point in scan.ranges:
        # convert to cartesian
        x = point * math.cos(angle)
        y = point * math.sin(angle)
        angle += scan.angle_increment

        if point != 0:
            points.append(Point(x, y, 0.0))

    return points


rospy.init_node('person_finder')

listener = tf.TransformListener()

minDist = rospy.get_param('min_dist', 0.1)
maxDist = rospy.get_param('max_dist', 2.0)
boxWidth = rospy.get_param('box_width', 2.0)


class PersonFinder(object):
    def __init__(self):
        super(PersonFinder, self).__init__()
        self.pubPoint = rospy.Publisher('/person', PointStamped, queue_size=10)
        self.pubVis = rospy.Publisher('/person_visualisation', Marker, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.scan_callback)
        self.header = Header()

    def publish_point(self, point):
        newheader = Header(stamp=self.header.stamp, frame_id='base_link')
        self.pubPoint.publish(PointStamped(header=newheader, point=point))
        self.pubVis.publish(Marker(id=0, type=Marker.SPHERE,
                                   header=newheader,
                                   pose=Pose(position=point),
                                   scale=Vector3(0.1, 0.1, 0.1),
                                   color=ColorRGBA(g=1, a=1)))

    def filter_points(self, points):
        points = [p for p in points if (minDist <= p.x <= maxDist and abs(p.y) <= boxWidth / 2)]
        return points

    def transform_points(self, points):
        return [
            listener.transformPoint('base_link', PointStamped(header=self.header, point=p)).point
            for p in points]

    def scan_callback(self, scan):
        self.header = scan.header

        points = laser_to_pointcloud(scan)
        points = self.transform_points(points)
        points = self.filter_points(points)

        num_points = len(points)
        if num_points == 0:
            return
        mean_x = sum([p.x for p in points]) / num_points
        mean_y = sum([p.y for p in points]) / num_points

        self.publish_point(Point(x=mean_x, y=mean_y))


if __name__ == '__main__':
    PersonFinder()
    rospy.spin()
