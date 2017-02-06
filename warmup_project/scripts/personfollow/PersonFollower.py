#! /usr/bin/env python

import rospy, tf, math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, Vector3, Pose
# from visualization_msgs.msg import Marker
# from std_msgs.msg import ColorRGBA

# from frames import modernizePoint

rospy.init_node('line_follower')

listener = tf.TransformListener()
broadcaster = tf.TransformBroadcaster()

goal_dist = rospy.get_param('~person_offset', default=1.0)
speed = rospy.get_param('~speed', default=.2)

# Radians to bias angle per meter of offset angle
kpDist = rospy.get_param('~kpDist', default=1.5)

# Radians/sec to turn per radian of angle error
kpAngle = rospy.get_param('~kpAngle', default=1.0)

subTopic = rospy.get_param('~topic', '/person')

timeout = rospy.get_param('~timeout', 2.0)


class PersonFollower(object):
    def __init__(self):
        super(PersonFollower, self).__init__()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber(subTopic, PointStamped, self.on_detect)
        self.point = (0, 0)
        self.lastMessageTime = rospy.Time.from_seconds(0)
        # self.pubVis = rospy.Publisher('/person_visualisation', Marker, queue_size=10)


    def on_detect(self, msg):
        """
        :type msg: PointStamped
        """
        local_point = PointStamped(header=msg.header, point=msg.point)
        # local_point = modernizePoint(listener, local_point, 'odom')
        local_point = listener.transformPoint('base_link', local_point)

        # self.pubVis.publish(Marker(id=3, type=Marker.SPHERE,
        #                            header=local_point.header,
        #                            pose=Pose(position=local_point.point),
        #                            scale=Vector3(0.2, 0.2, 0.2),
        #                            color=ColorRGBA(b=1, a=.5)))

        self.lastMessageTime = local_point.header.stamp
        self.point = (local_point.point.x, local_point.point.y)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

            if rospy.Time.now() - self.lastMessageTime < rospy.Duration.from_sec(timeout):
                # We have recent data

                dist = self.get_dist_from_person()
                angle = self.get_angle_from_person()

                dist_error = dist - goal_dist
                desired_angle = 0

                # broadcaster.sendTransform((0, 0, 0),
                #                           tf.transformations.quaternion_from_euler(0, 0, desired_angle),
                #                           rospy.Time.now(), 'desired_heading', 'base_link')

                angle_error = angle - desired_angle

                # if abs(dist_error) > 200:
                #     # Something is broken, abort!
                #     continue

                print dist_error, angle_error

                if abs(angle_error) > 0.5:
                    forward_speed = speed / 3.0
                else:
                    forward_speed = speed

                turn_speed = angle_error * kpAngle

                msg = Twist(linear=Vector3(x=forward_speed), angular=Vector3(z=turn_speed))

                self.pub.publish(msg)

            else:
                # All of our data is really old, stop the robot
                print "No data"
                self.pub.publish(Twist())


            # print dist, angle

    def get_dist_from_person(self):
        """
        Returns the perpendicular distance of the robot from the wall, in meters.
        :return:
        """
        return math.sqrt(self.point[0] ** 2 + self.point[1] ** 2)

    def get_angle_from_person(self):
        """
        Returns the angle from the wall in radians, with positive indicating the robot is going towards the wall (assuming wall to right)
        """
        return math.atan2(self.point[1], self.point[0])


if __name__ == '__main__':
    PersonFollower().run()
