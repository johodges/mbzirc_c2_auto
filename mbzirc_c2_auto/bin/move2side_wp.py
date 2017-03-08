#!/usr/bin/env python

"""move2side_wp.py - Version 1.0 2017-02-02
Author: Alan Lattimer

Moves into position on a side of the panel to detect wrenches

Subscribers:

Publishers:

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html

"""
import rospy
import rospkg
import sys
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np

if __name__ == '__main__':
    rospy.init_node('moveToSideWP', anonymous=True)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
