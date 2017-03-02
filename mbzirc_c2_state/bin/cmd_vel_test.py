#!/usr/bin/env python
import roslib
import rospy
import rospkg
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import numpy as np
from robot_mv_cmds import *


if __name__ == '__main__':
    rospy.init_node('cmd_vel_test', anonymous=True)
    #moveUGVvel(0.50,2.5, 'angular')
    #rospy.sleep(0.1)
    dist_to_move = 0.0;
    x_vel = 0.0;
    z_vel = 0.0;
    sleep_time = 0.1
    time_to_move = 10 #abs(dist_to_move/x_vel)

    vel_twist = Twist()
    vel_twist.linear.x = x_vel
    vel_twist.angular.z = z_vel

    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    ct_move = 0
    while ct_move*sleep_time < time_to_move:
        vel_pub.publish(vel_twist)
        ct_move = ct_move+1
        rospy.sleep(sleep_time)
    #vel_twist = Twist()
    #vel_pub.publish(vel_twist)
    #rospy.sleep(sleep_time)

