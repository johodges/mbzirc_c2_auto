#!/usr/bin/env python
import roslib
import rospy
import rospkg
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import numpy as np


if __name__ == '__main__':
    rospy.init_node('move_arm_pub', anonymous=True)
    goal_pub = rospy.Publisher("/move_arm/goal",Twist, queue_size=1)
    #ee_position = rospy.get_param('ee_position')
    ct = 0
    twist = Twist()
    twist.linear.x = 0.4
    twist.linear.y = -0.2
    twist.linear.z = 0.95
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    print twist
    while ct < 10:
        goal_pub.publish(twist)
        ct = ct+1
        rospy.sleep(0.1)

