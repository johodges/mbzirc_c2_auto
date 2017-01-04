#!/usr/bin/env python

"""findbox_velodyne.py - Version 1.0 2016-10-12
Author: Jonathan Hodges

This code searches a 3-D LIDAR scan for an object within a minimum and maximum
length bound. The LIDAR scan is segmented based on null returns and large
deviations between points.

Subscribers:
    /velodyne_points: 3-D point cloud from ROS

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

import roslib;
import sensor_msgs.point_cloud2 as pc2
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
from sensor_msgs.msg import Image, PointCloud2
import cv2
from cv_bridge import CvBridge, CvBridgeError
import urllib, base64
import os
import sys

def callback(data):
    gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    int_data = np.asarray(list(gen))
    x = int_data[:,0]
    y = int_data[:,1]
    z = int_data[:,2]

    mask = z > 0
    x = x[mask]
    y = y[mask]
    z = z[mask]

    ind = np.argsort(y)
    x = x[ind]
    y = y[ind]
    x_coord = x
    y_coord = y
    z = z[ind]

    rospy.logdebug("Total number of points: %f", np.shape(x))
    # Initialize parameters
    rate = rospy.Rate(10)
    scan_dist_thresh = 0.5  # Distance threshold to split obj into 2 obj.
    plot_data = False

    # Set max/min angle and increment
    #scan_min = data.angle_min
    #scan_max = data.angle_max
    #scan_inc = data.angle_increment

    # Build angle array
    #x = np.arange(scan_min,scan_max+scan_inc*0.1,scan_inc)

    # Pre-compute trig functions of angles
    #xsin = np.sin(x)
    #xcos = np.cos(x)

    # Apply a median filter to the range scans
    #y = sg.medfilt(data.ranges,1)

    # Calculate the difference between consecutive range values
    y_diff1 = np.power(np.diff(y),2)

    # Convert range and bearing measurement to cartesian coordinates
    #x_coord = y*xsin
    #y_coord = y*xcos

    # Compute difference between consecutive values in cartesian coordinates
    x_diff = np.power(np.diff(x_coord),2)
    y_diff = np.power(np.diff(y_coord),2)

    # Compute physical distance between measurements
    dist = np.power(y_diff,0.5)

    # Segment the LIDAR scan based on physical distance between measurements
    x2 = np.array(np.split(x, np.argwhere(
        dist > scan_dist_thresh).flatten()[1:]))
    y2 = np.array(np.split(y, np.argwhere(
        dist > scan_dist_thresh).flatten()[1:]))
    dist2 = np.array(np.split(dist, np.argwhere(
        dist > scan_dist_thresh).flatten()[1:]))
    x_coord2 = np.array(np.split(x_coord, np.argwhere(
        dist > scan_dist_thresh).flatten()[1:]))
    y_coord2 = np.array(np.split(y_coord, np.argwhere(
        dist > scan_dist_thresh).flatten()[1:]))
    #print "dist_mx/mn: ", np.max(dist), np.min(dist)
    # Loop through each segmented object
    for i in range(len(x2)):

        # Check if there are at least 4 points in an object (reduces noise)
        xlen = len(x2[i])-0
        if xlen > 1:

            # Calculate distance of this object
            dist2_sum = np.sum(dist2[i][1:xlen-1])
            #print dist2_sum

            # Check if this object is too small
            if dist2_sum > 0.25 and dist2_sum < 4:
                ang = np.arctan(np.median(y2[i])/np.median(x2[i]))
                dis = np.median(x2[i])/np.cos(ang)
                mn = min(x2[i][1:xlen])
                mx = max(x2[i][1:xlen])
                bearing = np.array([ang,dis], dtype=np.float32)

    # Check if bearing exists. Store [0,0] if no object was found
    if 'bearing' not in locals():
        bearing = np.array([0,0], dtype=np.float32)
    #print "bearing: ", bearing
    # Publish bearing to ROS on topic /detection
    pub = rospy.Publisher("/detection",numpy_msg(Floats), queue_size=1)
    pub.publish(bearing)

    # If we want to plot the LIDAR scan, open the plot environment
    if plot_data:
        plt.figure(1)
        for i in range(len(x2)):
            xlen = len(x2[i])-0
            if xlen > 1:
                dist2_sum = np.sum(dist2[i][1:xlen-1])
                if dist2_sum < 0.25:
                    if plot_data:
                        plt.plot(y2[i][1:xlen],x2[i][1:xlen],'k-',
                            linewidth=2.0)
                else:
                    if dist2_sum > 4:
                        if plot_data:
                            plt.plot(y2[i][1:xlen],x2[i][1:xlen],'b-',
                                linewidth=2.0)
                    else:
                        if plot_data:
                            plt.plot(y2[i][1:xlen],x2[i][1:xlen],'r-',
                                linewidth=2.0)
        plt.ylim([0,20])
        plt.xlim([-5,5])
        plt.gca().invert_xaxis()
        plt.xlabel('Left of robot [m] ')
        plt.ylabel('Front of robot [m]')
        plt.title('Laser Scan')
        imgdata = StringIO.StringIO()
        plt.savefig(imgdata, format='png')
        imgdata.seek(0)
        img_array = np.asarray(bytearray(imgdata.read()), dtype=np.uint8)
        im = cv2.imdecode(img_array, 1)
        bridge = CvBridge()
        image_output = rospy.Publisher("/output/keyevent_image",Image, 
            queue_size=1)
        image_output.publish(bridge.cv2_to_imgmsg(im, "bgr8"))
        plt.close()
    pass



def laser_listener():
    '''Entry point for the file.  Subscribe to lase scan topic and wait
    '''
    pass
    rospy.init_node('findbox_velodyne', anonymous=True)
    rospy.Subscriber("/velodyne_points",PointCloud2,callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo('Looking for object...')
    laser_listener()
