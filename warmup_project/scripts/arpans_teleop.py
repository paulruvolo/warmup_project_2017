#!/usr/bin/env python

#This node allows the user to drive with the IJKL keys (I forward, K reverse, J left, L right)
#Pressing any key that is not IJKL stops the robot
import rospy
from geometry_msgs.msg import Twist,Vector3
import tty
import select
import sys
import termios

def getKey():
    """gets a pressed key"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
key = None


#start node
rospy.init_node('arpan_driver')
#publisher to publish to /vmd_vel
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
r = rospy.Rate(2)



while not rospy.is_shutdown():
    #Get pressed Key
    key = getKey()
    #Depending on key, write x and z of robot's twist msg
    if key == 'i':
        x = 1
        z= 0
    elif key =='k':
        x = -1
        z = 0
    elif key == 'l':
        z = -1
        x = 0
    elif key == 'j':
        z = +1
        x = 0
    #Shutdown on ctrl-c
    elif key == '\x03':
        rospy.signal_shutdown("Shutdown Commanded")
    #any other key stops motion
    else:
        x = 0
        z = 0
    print key
    #create Twist Msg
    velcmd = Twist(Vector3(x,0,0),Vector3(0,0,z))
    r.sleep()
    pub.publish(velcmd)