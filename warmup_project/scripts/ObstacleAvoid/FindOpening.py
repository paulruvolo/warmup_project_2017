#!/usr/bin/env python

import rospy
import tf
import math
import time

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose, Point, PointStamped, PoseStamped
from std_msgs.msg import ColorRGBA, Header


def laser_to_pointcloud(scan, new_frame='base_link'):
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
            point = PointStamped(header=scan.header, point=Point(x, y, 0.0))
            points.append(listener.transformPoint(new_frame, point).point)

    return points


rospy.init_node('obstacle_avoider')

listener = tf.TransformListener()

obstaclePower = rospy.get_param('~obstacle_power', .1)
obstacleDistThresh = rospy.get_param('~obstacle_dist_thresh', .3)  # meters
obstacleScalingExp = rospy.get_param('~obstacle_scaling_exp', 5)

goalPower = rospy.get_param('~goal_power', 1)

time.sleep(1)
print('Starting')


class OpeningFinder(object):
    def __init__(self):
        super(OpeningFinder, self).__init__()
        self.pubPoint = rospy.Publisher('/person', PointStamped, queue_size=10)
        self.pubVis = rospy.Publisher('/person_visualisation', Marker, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.scan_callback)
        self.goalSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.ultimate_goal = PointStamped(header=Header(frame_id='odom'), point=Point(x=1000))

    def goal_callback(self, goal):
        header = goal.header
        header.stamp = rospy.Time()

        point = Point(x=goal.pose.position.x, y=goal.pose.position.y)
        self.ultimate_goal = PointStamped(header=header, point=point)

    def avoidance_magnitude(self, dist):
        return obstaclePower / ((dist/obstacleDistThresh) ** obstacleScalingExp)

    def publish_dest(self, force, header):
        self.pubPoint.publish(PointStamped(header=header, point=Point(x=force.x, y=force.y)))
        self.publish_vector(force, header, id=2112, color=ColorRGBA(g=1, a=1))

    def publish_vector(self, vect, header, id=0, color=ColorRGBA(g=1, a=1)):
        self.pubVis.publish(Marker(id=id, type=Marker.ARROW,
                                   header=header,
                                   scale=Vector3(x=.1, y=.4),
                                   color=color,
                                   points=[Point(), Point(x=vect.x, y=vect.y)]))

    def pointForce(self, point):
        angleToPoint = math.atan2(point.y, point.x)
        distToPoint = math.sqrt(point.x ** 2 + point.y ** 2)
        mag = self.avoidance_magnitude(distToPoint)
        return Vector3(x=-math.cos(angleToPoint) * mag, y=-math.sin(angleToPoint) * mag)

    def goalForce(self):
        goal = listener.transformPoint('base_link', self.ultimate_goal)
        angleToGoal = math.atan2(goal.point.y, goal.point.x)
        return Vector3(x=math.cos(angleToGoal) * goalPower, y=math.sin(angleToGoal) * goalPower)

    def scan_callback(self, scan):
        points = laser_to_pointcloud(scan)

        new_header = Header(frame_id='base_link', stamp=scan.header.stamp)

        force = self.goalForce()

        self.publish_vector(force, new_header, id=123123, color=ColorRGBA(b=1, a=1))

        for p in points:
            point_force = self.pointForce(p)
            force.x += point_force.x
            force.y += point_force.y

        self.publish_dest(force, new_header)


if __name__ == '__main__':
    OpeningFinder()
    rospy.spin()
