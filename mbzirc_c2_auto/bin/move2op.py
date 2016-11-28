#!/usr/bin/env python

""" move2grasp.py - Version 1.0 2016-10-12

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
from sensor_msgs.msg import Image
import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
from decimal import *
import tf
import math
import random

class move2op():
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('move2op', anonymous=True)
        
        # Enable shutdown in rospy (This is important so we cancel any move_base goals
        # when the node is killed)
        rospy.on_shutdown(self.shutdown) # Set rospy to execute a shutdown function when exiting

        # Store camera parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080

        # Set up ROS subscriber callback routines
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/mybot/camera1/image_raw",Image,self.callback)

    def shutdown(self):
        rospy.sleep(1)

    # callback_wrench is used to store the wrench topic into the class to be
    # referenced by the other callback routines.
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    #if random.random() < 0.1:
    #    rospy.set_param('smach_state','wrenchFell')
    #else:
        rospy.set_param('smach_state','wrenchOnValve')
        rospy.sleep(5)

if __name__ == '__main__':
    try:
        move2op()
#        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("move2op finished.")

