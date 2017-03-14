#!/usr/bin/env python

"""key_pulisher.py - Version 1.0 2016-12-27
Author: Alan Lattimer

This reads keystrokes from the stdin stream and publishes them to a topic for
use in moving the robot

Publishers:
    /keys : keystrokes from stdin stream

Acknowledgments:
    This code was based on Example 8.1 of 'Programming Robots with ROS', M. Quigley,
    B. Gerkey, and W. D. Smart, 2015

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html

"""

import sys
import select
import tty
import termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    key_pub = rospy.Publisher('keys',String,queue_size=1)
    rospy.init_node("key_publisher")
    rate = rospy.Rate(100)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    rospy.loginfo("Publishing keystrokes to 'keys' topic.")

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("SMACH initialized")

    rospy.loginfo('Ending key_publisher')
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
