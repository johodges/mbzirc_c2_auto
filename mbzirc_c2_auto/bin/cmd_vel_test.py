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
    moveUGVvel(0.5, 2.5, move_type='angular')
    rospy.sleep(0.1)

