#! /usr/bin/env python

import rospy, tf, numpy, math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, Vector3

rospy.init_node('line_follower')

listener = tf.TransformListener()
broadcaster = tf.TransformBroadcaster()

goal_dist = rospy.get_param('~wall_offset', default=1.0)
speed = rospy.get_param('~speed', default=.2)

# Radians to bias angle per meter of offset angle
kpDist = rospy.get_param('~kpDist', default=1.5)

# Radians/sec to turn per radian of angle error
kpAngle = rospy.get_param('~kpAngle', default=1.0)

subTopic = rospy.get_param('~topic', '/person')

class LineFollower(object):
    def __init__(self):
        super(LineFollower, self).__init__()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber(subTopic, PointStamped, self.on_detect)
        self.point = (0,0)

    def on_detect(self, msg):
        """
        :type msg: PointStamped
        """
        local_point = listener.transformPoint('base_link', PointStamped(header=msg.header, point=pt))
        self.point = (local_point.point.x, local_point.point.y)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

            if self.points[0] == self.points[1]:
                continue

            dist = self.get_dist_from_person()
            angle = self.get_angle_from_person()

            dist_error = dist - goal_dist
            desired_angle = dist_error * kpDist

            broadcaster.sendTransform((0, 0, 0),
                                      tf.transformations.quaternion_from_euler(0, 0, desired_angle),
                                      rospy.Time.now(), 'desired_heading', 'base_link')

            angle_error = angle - desired_angle

            turn_speed = angle_error * kpAngle

            msg = Twist(linear=Vector3(x=speed), angular=Vector3(z=turn_speed))

            self.pub.publish(msg)

            print  dist, angle

    def get_dist_from_person(self):
        """
        Returns the perpendicular distance of the robot from the wall, in meters.
        :return:
        """
        # taken from https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
        ps = self.points

        return (ps[1][1] * ps[0][0] - ps[1][0] * ps[0][1]) / \
               math.sqrt((ps[0][1] - ps[1][1]) ** 2 + (ps[0][0] - ps[1][0]) ** 2)

    def get_angle_from_person(self):
        """
        Returns the angle from the wall in radians, with positive indicating the robot is going towards the wall (assuming wall to right)
        """
        ps = self.points
        return math.atan2(ps[1][1] - ps[0][1], ps[1][0] - ps[0][0])


if __name__ == '__main__':
    LineFollower().run()
