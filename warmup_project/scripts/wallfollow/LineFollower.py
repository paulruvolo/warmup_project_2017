#! /usr/bin/env python
"""
This node listens to incomming messages on the /detect channel.
Those messages must be of type visualization_msgs/Marker with
type == LINE_STRIP and two points. These points are interpreted
as a directed infinite line the robot is intended to follow, with
constant lateral offset of (by default) 1m.

Note that the direction the robot wants to follow is denoted by the order of the points
in the incoming message, with the robot always travelling in the direction from the first
point to the second.
Similarly, to follow a wall on the left, the offset must be negative.

TODO: Use custom message instead of Marker
TODO: Prevent angle from wrapping when given very large offsets
"""
import math

import rospy
import tf
from geometry_msgs.msg import PointStamped, Vector3
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

rospy.init_node('line_follower')

listener = tf.TransformListener()
broadcaster = tf.TransformBroadcaster()

goal_dist = rospy.get_param('~wall_offset', default=1.0)
speed = rospy.get_param('~speed', default=.2)

# Radians to bias angle per meter of offset angle
kpDist = rospy.get_param('~kpDist', default=1.5)

# Radians/sec to turn per radian of angle error
kpAngle = rospy.get_param('~kpAngle', default=1.0)

subTopic = rospy.get_param('~topic', 'detect')

class LineFollower(object):
    def __init__(self):
        super(LineFollower, self).__init__()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber(subTopic, Marker, self.on_detect)
        self.points = [[0, 0], [0, 0]]

    def on_detect(self, msg):
        """
        ROS callback for incoming wall detect
        Simply stores values, does no meaningful processing
        :param msg: The incomming Marker (must be LINE_STRIP with two points
        :type msg: Marker
        """
        if msg.type != Marker.LINE_STRIP or len(msg.points) != 2:
            return

        for i, pt in enumerate(msg.points):
            local_point = listener.transformPoint('base_link', PointStamped(header=msg.header, point=pt))
            self.points[i] = [local_point.point.x, local_point.point.y]

    def run(self):
        """
        Loops until program ends, determining commands to send to motors
        :return: None
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

            if self.points[0] == self.points[1]:
                continue

            dist = self.get_dist_from_wall()
            angle = self.get_angle_from_wall()

            # Calculate the desired driving angle of the robot given the current distance
            dist_error = dist - goal_dist
            desired_angle = dist_error * kpDist

            broadcaster.sendTransform((0, 0, 0),
                                      tf.transformations.quaternion_from_euler(0, 0, desired_angle),
                                      rospy.Time.now(), 'desired_heading', 'base_link')

            # Calculate the desired turning speed
            angle_error = angle - desired_angle

            turn_speed = angle_error * kpAngle

            msg = Twist(linear=Vector3(x=speed), angular=Vector3(z=turn_speed))

            self.pub.publish(msg)

            print dist, angle

    def get_dist_from_wall(self):
        """
        :return: The perpendicular distance of the robot from the wall, in meters.
        """
        # taken from https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
        ps = self.points

        return (ps[1][1] * ps[0][0] - ps[1][0] * ps[0][1]) / \
               math.sqrt((ps[0][1] - ps[1][1]) ** 2 + (ps[0][0] - ps[1][0]) ** 2)

    def get_angle_from_wall(self):
        """
        :return: The angle from the wall in radians, with positive indicating the robot is going towards the wall (assuming wall to right)
        """
        ps = self.points
        return math.atan2(ps[1][1] - ps[0][1], ps[1][0] - ps[0][0])


if __name__ == '__main__':
    LineFollower().run()
