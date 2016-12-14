#!/usr/bin/env python

""" findbox.py - Version 1.0 2016-10-12

    Initial version by Jonathan Hodges.

    This code searches a 2-D LIDAR scan for an object within
    a minimum and maximum length bound. The LIDAR scan is
    segmented based on null returns and large deviations between
    points. 

    Inputs:
        /scan     - 2-D LIDAR scan from ROS
        plot_flag - If zero, do not plot LIDAR scan. Plotting the
                    scan slows down the code.
    Outputs:
        /detection- array containing [angle,distance] to the median of
                    the detected object in local coordinates. Contains
                    [0,0] if no object is detected.

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

"""
This callback runs each time a LIDAR scan is obtained from
the /scan topic in ROS. Returns a topic /detection. If no
object is found, /detection = [0,0]. If an object is found,
/detection = [angle,distance] to median of scan.
"""

def callback(data):
    # Initialize parameters
    rate = rospy.Rate(10)
    scan_dist_thresh = 0.1  # Distance threshold to split obj into 2 obj.
    plot_flag = 0

    # Set max/min angle and increment
    x = np.arange(1.66,-1.6525,-0.0029146999)

    # Pre-compute trig functions of angles
    xsin = np.sin(x)
    xcos = np.cos(x)

    # Apply a median filter to the range scans
    y = sg.medfilt(data.ranges,1)

    # Calculate the difference between consecutive range values
    y_diff1 = np.power(np.diff(y),2)

    # Convert range and bearing measurement to cartesian coordinates
    x_coord = y*xsin
    y_coord = y*xcos

    # Compute difference between consecutive values in cartesian coordinates
    x_diff = np.power(np.diff(x_coord),2)
    y_diff = np.power(np.diff(y_coord),2)

    # Compute physical distance between measurements
    dist = np.power(x_diff+y_diff,0.5)

    # Segment the LIDAR scan based on the physical distance between measurements
    x2 = np.array(np.split(x, np.argwhere(dist > scan_dist_thresh).flatten()[1:]))
    y2 = np.array(np.split(y, np.argwhere(dist > scan_dist_thresh).flatten()[1:]))
    dist2 = np.array(np.split(dist, np.argwhere(dist > scan_dist_thresh).flatten()[1:]))
    x_coord2 = np.array(np.split(x_coord, np.argwhere(dist > scan_dist_thresh).flatten()[1:]))
    y_coord2 = np.array(np.split(y_coord, np.argwhere(dist > scan_dist_thresh).flatten()[1:]))

    # If we want to plot the LIDAR scan, open the plot environment
    if plot_flag == 1:
        plt.figure(1)
        #plt.cla()

    # Loop through each segmented object
    for i in range(len(x2)):

        # Check if there are at least 4 points in an object (reduces noise)
        xlen = len(x2[i])-0
        if xlen > 4:

            # Calculate distance of this object
            dist2_sum = np.sum(dist2[i][1:xlen-1])

            # Check if this object is too small
            if dist2_sum < 0.25:
                # If we want to plot, plot small object
                if plot_flag == 1:
                    plt.plot(x2[i][1:xlen],y2[i][1:xlen],'k-',linewidth=2.0)
            else:

                # Check if this object is too big
                if dist2_sum > 1.5:
                    # If we want to plot, plot large object
                    if plot_flag == 1:
                        plt.plot(x2[i][1:xlen],y2[i][1:xlen],'b-',linewidth=2.0)
                else:
                    # If we want to plot, plot just right object
                    if plot_flag == 1:
                        plt.plot(x2[i][1:xlen],y2[i][1:xlen],'r-',linewidth=2.0)

                    # Find range and bearing of median of object
                    ang = np.median(x2[i])
                    dis = np.median(y2[i])
                    mn = min(y2[i][1:xlen])
                    mx = max(y2[i][1:xlen])
                    bearing = np.array([ang,dis], dtype=np.float32)

    # Check if bearing exists. Store [0,0] if no object was found
    if 'bearing' not in locals():
        bearing = np.array([0,0], dtype=np.float32)

    # Publish bearing to ROS on topic /detection
    pub = rospy.Publisher("/detection",numpy_msg(Floats), queue_size=1)
    pub.publish(bearing)

    # If we want to plot, make it prettier
    if plot_flag == 1:
        plt.ylim([0,20])
        plt.xlim([-5,5])
        plt.xlabel('Left of robot [m] ')
        plt.ylabel('Front of robot [m]')
        plt.title('Laser Scan')
        plt.draw()
        plt.pause(0.001)
    pass

def laser_listener():
    pass
    rospy.init_node('findbox', anonymous=True)
    rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo('Looking for object...')
    laser_listener()
