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
    ee_position = rospy.get_param('ee_position', [1.0, -0.08, 0.45])
    ct = 0
    twist = Twist()
    twist.linear.x = ee_position[0]
    twist.linear.y = ee_position[1]
    twist.linear.z = ee_position[2]

    print twist
    while ct < 1000:
        goal_pub.publish(twist)
        ct = ct+1
        rospy.sleep(0.1)

