#!/usr/bin/env python

"""findbox.py - Version 1.0 2016-10-12
Author: Jonathan Hodges

This code searches a 2-D LIDAR scan for an object within a minimum and maximum
length bound. The LIDAR scan is segmented based on null returns and large
deviations between points.

Subscribers:
    /scan: 2-D LIDAR scan from ROS

Publishers:
    /detection: array containing [angle,distance] to the median of the detected
        object in local coordinates. Contains [0,0] if no object is detected.
    /output/keyevent_image: image containing key events in the challenge. This
        code publishes the segmented LIDAR scan.

Attributes:
    plot_data - If False, do not plot LIDAR scan. Plotting the scan slows down
        the code.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html

"""

import roslib; roslib.load_manifest('urg_node')
import rospy
import sensor_msgs.msg
import matplotlib.pyplot as plt
import numpy as np
import scipy as sc
import scipy.signal as sg
import scipy.misc as ms
import scipy.spatial.distance as scd
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import StringIO
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import urllib, base64
import os
import sys

def laser_listener():
    '''Entry point for the file.  Subscribe to lase scan topic and wait
    '''
    pass
    rospy.init_node('findbox', anonymous=True)
    pub = rospy.Publisher("/detection",numpy_msg(Floats), queue_size=1)
    bearing = np.array([0,0], dtype=np.float32)
    while True:
        pub.publish(bearing)
        print "bearing: ", bearing
        rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.loginfo('Looking for object...')
    laser_listener()
    rospy.spin()
