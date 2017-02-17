#!/usr/bin/env python

"""
FindOpening implements a gradient-field based navigation approach
based on the data from the laser scanner.

It subscribes to /stable_scan and /move_base_simple/goal
and publishes to person (Point) and person_visualization (Marker)

If the node has not yet been given a goal, it will attempt to drive torward +x
in the odom coordinate frame.

TODO: Signal when the goal has been reached
"""

import math
import time

import rospy
import tf
from geometry_msgs.msg import Vector3, Point, PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


def laser_to_pointcloud(scan, new_frame='base_link'):
    """
    Converts a LaserScan into a list of points into new_frame.

    TODO: Use Paul's published pointcloud topic instead
    :type scan: LaserScan
    """
    points = []
    angle = scan.angle_min
    for point in scan.ranges:
        # convert to cartesian
        x = point * math.cos(angle)
        y = point * math.sin(angle)
        angle += scan.angle_increment

        # Transform the point into the correct tf frame
        if point != 0:
            point = PointStamped(header=scan.header, point=Point(x, y, 0.0))
            points.append(listener.transformPoint(new_frame, point).point)

    return points


rospy.init_node('obstacle_avoider')

listener = tf.TransformListener()

# Force multiplier for obstacles
obstaclePower = rospy.get_param('~obstacle_power', .1)

# Distance (meters) in which the repulsion power is the value given above
obstacleDistThresh = rospy.get_param('~obstacle_dist_thresh', .6)

# Scaling exponent controlling how quickly obstacle power rises inside the threshold specified above
obstacleScalingExp = rospy.get_param('~obstacle_scaling_exp', 2)  #

# Absolute force applied in the direction of the goal.
goalPower = rospy.get_param('~goal_power', 1)

# Allow the tf listener to gather sufficient data to process incoming scans.
time.sleep(1)
print('Starting')


class OpeningFinder(object):
    def __init__(self):
        """
        Sets up subscribers and publishers, and initiatilizes the goal at +x=1000
        """
        super(OpeningFinder, self).__init__()
        self.pubPoint = rospy.Publisher('person', PointStamped, queue_size=10)
        self.pubVis = rospy.Publisher('person_visualisation', Marker, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.scan_callback)
        self.goalSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.ultimate_goal = PointStamped(header=Header(frame_id='odom'), point=Point(x=1000))

    def goal_callback(self, goal):
        """
        ROS callback for goalSub
        :type goal: PoseStamped
        :return: None
        """
        header = goal.header
        header.stamp = rospy.Time()

        point = Point(x=goal.pose.position.x, y=goal.pose.position.y)
        self.ultimate_goal = PointStamped(header=header, point=point)

    def avoidance_magnitude(self, dist):
        """
        Computes the magnitude of the repulsion force from a point at a given distance
        :param float dist: absolute distance, in meters
        :return:
        """
        return obstaclePower / ((dist/obstacleDistThresh) ** obstacleScalingExp)

    def publish_dest(self, force, header):
        """
        Publishes and visualizes the output vector
        :param Vector3 force: the total force acting on the robot
        :param Header header:
        """
        self.pubPoint.publish(PointStamped(header=header, point=Point(x=force.x*1000, y=force.y*1000)))
        self.visualize_vector(force, header, id=2112, color=ColorRGBA(g=1, a=1))

    def visualize_vector(self, vect, header, id=0, color=ColorRGBA(g=1, a=1)):
        """
        Helper function for publishing a
        :param Vector3 vect: Vector to visualize
        :param Header header:
        :param int id: Should be unique for each type of marker visualized
        :return: None
        """
        self.pubVis.publish(Marker(id=id, type=Marker.ARROW,
                                   header=header,
                                   scale=Vector3(x=.1, y=.4),
                                   color=color,
                                   points=[Point(), Point(x=vect.x, y=vect.y)]))

    def pointForce(self, point):
        """
        Calculates the vector repulsion force from a given Point from the laser
        :param Point point: the scanned point (in base_link coordinate frame)
        :return Vector3: The repulsion force acting on the robot
        """
        angleToPoint = math.atan2(point.y, point.x)
        distToPoint = math.sqrt(point.x ** 2 + point.y ** 2)
        mag = self.avoidance_magnitude(distToPoint)
        return Vector3(x=-math.cos(angleToPoint) * mag, y=-math.sin(angleToPoint) * mag)

    def goalForce(self):
        """
        Calculates the vector force attracting the robot to the goal
        :return Vector3:
        """
        goal = listener.transformPoint('base_link', self.ultimate_goal)
        angleToGoal = math.atan2(goal.point.y, goal.point.x)
        return Vector3(x=math.cos(angleToGoal) * goalPower, y=math.sin(angleToGoal) * goalPower)

    def scan_callback(self, scan):
        """
        ROS callback for laser scan
        Calculates total force and publishes the new command point
        All of the substantive work of the node happens here.
        :type scan: LaserScan
        """
        points = laser_to_pointcloud(scan)

        new_header = Header(frame_id='base_link', stamp=scan.header.stamp)

        force = self.goalForce()

        self.visualize_vector(force, new_header, id=123123, color=ColorRGBA(b=1, a=1))

        for p in points:
            point_force = self.pointForce(p)
            force.x += point_force.x
            force.y += point_force.y

        self.publish_dest(force, new_header)


if __name__ == '__main__':
    OpeningFinder()
    rospy.spin()
