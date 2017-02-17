#!/usr/bin/env python

from enum import Enum
import rospy
import tf
import time
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Vector3
from neato_node.msg import Bump
from std_msgs.msg import String, Header

rospy.init_node('state_machine')
listener = tf.TransformListener()

cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
avoid_dest_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
state_pub = rospy.Publisher('state', String, queue_size=10)

rospy.on_shutdown(lambda: cmd_pub.publish(Twist()))

transition_timeout = rospy.Duration.from_sec(2)
backup_dist = 0.3
backup_speed = 0.1

class State(Enum):
    PERSON_FOLLOW = 1
    WALL_FOLLOW = 2
    OBSTACLE_AVOID = 3
    BACKING_UP = 4


class StateMachine(object):
    def __init__(self):
        super(StateMachine, self).__init__()
        self.bumped = False
        self.state = State.PERSON_FOLLOW
        self.lastTransitionTime = rospy.Time()
        rospy.Subscriber('/bump', Bump, self.bumpCallback)
        self.makeSubscriber(State.PERSON_FOLLOW, 'personfollow')
        self.makeSubscriber(State.WALL_FOLLOW, 'wallfollow')
        self.makeSubscriber(State.OBSTACLE_AVOID, 'obstacleavoid')

    def makeSubscriber(self, state, namespace):
        def cb(msg):
            if self.state == state:
                cmd_pub.publish(msg)

        return rospy.Subscriber(namespace + '/cmd_vel', Twist, cb)

    def bumpCallback(self, msg):
        """
        :type msg: Bump
        """
        self.bumped = msg.leftFront or msg.leftSide or msg.rightFront or msg.rightSide

    def checkTransitions(self):
        if (rospy.Time.now() - self.lastTransitionTime) < transition_timeout:
            return
        if self.state == State.PERSON_FOLLOW:
            if self.bumped:
                self.transitionTo(State.WALL_FOLLOW)

        elif self.state == State.WALL_FOLLOW:
            if self.bumped:
                dest = listener.transformPose('odom',
                                              PoseStamped(header=Header(frame_id='base_link'),
                                                          pose=Pose(position=Point(x=2.0))))
                avoid_dest_pub.publish(dest)
                self.transitionTo(State.OBSTACLE_AVOID)

        elif self.state == State.OBSTACLE_AVOID:
            if self.bumped:  # TODO: make this when reached destination
                self.transitionTo(State.WALL_FOLLOW)

        else:
            print "Unknown State: This shouldn't happen"

    def transitionTo(self, newstate):
        print 'transitioning to', newstate
        state_pub.publish(String(str(newstate)))
        self.state = newstate
        self.lastTransitionTime = rospy.Time.now()
        self.backup()

    def backup(self):
        print 'backing up'
        oldstate = self.state
        self.state = State.BACKING_UP
        cmd_pub.publish(Twist(linear=Vector3(x=-backup_speed)))
        time.sleep(backup_dist/backup_speed)
        cmd_pub.publish(Twist())
        self.state = oldstate

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            r.sleep()
            self.checkTransitions()


if __name__ == '__main__':
    StateMachine().run()
