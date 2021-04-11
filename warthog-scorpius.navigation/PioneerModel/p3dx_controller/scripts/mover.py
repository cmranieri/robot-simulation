#!/usr/bin/env python
import getch
import roslib
import rospy

from geometry_msgs.msg import Twist



KEY_UP = 65
KEY_DOWN = 66
KEY_RIGHT = 67
KEY_LEFT = 68
USER_QUIT = 100

MAX_FORWARD = 0.8
MAX_LEFT = 0.5
MIN_FORWARD = -0.8
MIN_LEFT = -0.5

forward = 0.0
left = 0.0
keyPress = 0


while(keyPress != USER_QUIT):
    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('p3dx_mover')
    twist = Twist()

    keyPress = getch.getArrow()

    if((keyPress == KEY_UP) and (forward <= MAX_FORWARD)):
        forward += 0.05
    elif((keyPress == KEY_DOWN) and (forward >= MIN_FORWARD)):
        forward -= 0.05
    elif((keyPress == KEY_LEFT) and (left <= MAX_LEFT)):
        left += 0.1
    elif((keyPress == KEY_RIGHT) and (left >= MIN_LEFT)):
        left -= 0.1

    twist.linear.x = forward
    twist.angular.z = left
    pub.publish(twist)


pub = rospy.Publisher('/cmd_vel', Twist)
rospy.init_node('p3dx_mover')
twist = Twist()
twist.linear.x = 0.0
twist.angular.z = 0.0

pub.publish(twist)
exit()
	
