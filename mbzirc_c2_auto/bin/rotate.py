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
import rospkg
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
from decimal import *
import tf
import math
import random

if __name__ == '__main__':
    rospy.init_node('rotate')
    print "I am pretending to rotate."
    rospy.sleep(1)
    rnd = random.random()
    if rnd < 0.1:
        rospy.set_param('smach_state','wrenchFell')
    else:
        if rnd < 0.3:
            rospy.set_param('smach_state','cantTurnValve')
        else:
            rospy.set_param('smach_state','turnedValve')
    rospy.signal_shutdown('Ending node.')

