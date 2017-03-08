#!/usr/bin/env python

"""move2wrenches.py - Version 1.0 2017-02-02
Author: Alan Lattimer

Moves the UGV into position to prepare to grab the wrenches

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
    rospy.init_node('moveToWrenches', anonymous=True)


    rospy.set_param('smach_state','oriented')
    rospy.loginfo('Finished scanning the current side.')
    rospy.signal_shutdown('Ending node.')
