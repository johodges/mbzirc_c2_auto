#!/usr/bin/env python

""" grasp.py - Version 1.0 2016-10-12

    This software chooses the left most wrench in an RGB image and outputs an
    estimate of its 3D location in space relative to the camera [x,y,z]
    Made by Jonathan Hodges

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('grasp')
    try:
        physical_robot = rospy.get_param('physical_robot')
    except:
        physical_robot = 'False'
    if not physical_robot:
        pub = rospy.Publisher('gripper/cmd_vel', Twist, queue_size = 1)
        twist = Twist()
        speed = .5; turn = 1
        x = 0; y = 0; z = 0;
        th = 1 # To open gripper (1) use th = 1
        twist.linear.x = x*speed;
        twist.linear.y = y*speed;
        twist.linear.z = z*speed;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th*turn

        ct = 0
        rest_time = 0.1
        tot_time = 1
    
        while ct*rest_time < tot_time:
            pub.publish(twist)
            rospy.sleep(0.1)
            ct = ct+1
    
        th = -1
        twist.angular.z = th*turn
        ct = 0
        tot_time = 3

        while ct*rest_time < tot_time:
            pub.publish(twist)
            rospy.sleep(0.1)
            ct = ct+1

    rospy.set_param('smach_state','wrenchGrasped')
    rospy.signal_shutdown('Ending node.')

