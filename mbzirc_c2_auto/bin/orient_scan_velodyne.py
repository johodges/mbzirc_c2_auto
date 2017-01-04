#!/usr/bin/env python

"""orient_scan.py - Version 1.0 2016-10-12
Author: Jonathan Hodges

This code searches a 3-D LIDAR scan for an object within a minimum and maximum
length bound. The LIDAR scan is segmented based on null returns and large
deviations between points.

Subscribers:
    /velodyne_points: 3-D point cloud from ROS
    /scan: 2-D LIDAR scan from ROS

Publishers:
    /bearing: array containing [angle,x_med,xmn,xmx,ymn,ymx,target_x,target_y]

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
import sensor_msgs.point_cloud2 as pc2
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import numpy as np
import scipy as sc
import scipy.signal as sg
import scipy.misc as ms
import scipy.spatial.distance as scd
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import math

class laser_listener():
    """MBZIRC Challenge 2 orient_scan_velodyne

    This code searches a 3-D LIDAR scan for an object within a minimum and
    maximum length bound. The LIDAR scan is segmented based on null returns and
    large deviations between points.

    Attributes:
        self.pub: Publisher for bearing topic

    Subscribers:
        /velodyne_points: 3-D LIDAR scan from ROS

    Publishers:
        /bearing: array containing
            [angle,x_med,xmn,xmx,ymn,ymx,target_x,target_y]
"""

    def __init__(self):
        """This initializes the publishers and subscribers in the class.
        """
        # Name this node, it must be unique
        rospy.init_node('orient_scan_velodyne', anonymous=True)

        # Set up ROS subscriber callback routines
        rospy.Subscriber("/velodyne_points",PointCloud2,self.callback, queue_size=1)
        self.pub = rospy.Publisher("/bearing",numpy_msg(Floats), queue_size=1)

    def callback(self, data):
        """callback (/velodyne_points) is used to segment a laser scan into
        continuous segments and identify if the robot is normal to the object
        scan
        """
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

        # Calculate the difference between consecutive range values
        y_diff1 = np.power(np.diff(y),2)

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

        # Loop through each segmented object
        for i in range(len(x2)):

            # Check if there are at least 4 points in an object (reduces noise)
            xlen = len(x2[i])-0
            if xlen > 1:
    
                # Calculate distance of this object
                dist2_sum = np.sum(dist2[i][1:xlen-1])
    
                # Check if this object is too small
                if dist2_sum > 0.25 and dist2_sum < 4:
                    # Find median angle of scan
                    med_check = np.median(y2[i])
                    idx = np.argwhere(y2[i] == med_check)
                    # If no median exists because an even number of entries, ignore
                    # the first entry to make it an odd number of entries
                    if len(idx) == 0:
                        yA = np.median(y2[i][1:xlen])
                        idx = np.argwhere(y2[i] == yA)
                        xA = x2[i][idx]
                        ang = np.arctan(yA/xA)
                        dis = xA/np.cos(ang)
                    else:
                        yA = np.median(y2[i])
                        idx = np.argwhere(y2[i] == yA)
                        xA = x2[i][idx]
                        ang = np.arctan(yA/xA)
                        dis = xA/np.cos(ang)
                    mxmx = max(iii for iii in x_coord2[i] if iii < 30)
                    if mxmx > 0.5:
                        # Store minimum and maximum extents of scan
                        xmn = min(x_coord2[i][1:xlen])
                        xmx = max(x_coord2[i][1:xlen])
                        ymn = min(y_coord2[i][1:xlen])
                        ymx = max(y_coord2[i][1:xlen])
                        # Store physical (x,y) coordinates in local system of median
                        # angle of scan
                        idx2 = np.argwhere(x_coord2[i] == xmn)
                        xB = x_coord2[i][idx2]
                        yB = y_coord2[i][idx2]

                        # Determine which way the robot needs to rotate based on the
                        # physical orientations of the median and minimum scans
                        m = (yB-yA)/(xB-xA)
                        m2 = 1/m
                        theta2 = math.atan(-m2/1)
                        b2 = yA-m2*xA
                        d = 2
                        t1 = -2*xA-2*m2*yA+m2*m2+2*b2*m2
                        t2 = xA*xA+yA*yA-2*b2*yA+b2*b2-9

                        xc1 = pow(-t1+(t1*t1-4*t2),0.5)/2
                        xc2 = pow(-t1-(t1*t1-4*t2),0.5)/2
                        yc1 = m2*xc1+b2
                        yc2 = m2*xc2+b2

                        e = pow(xA*xA+yA*yA,0.5)
                        xc3 = xA-d*np.cos(theta2)
                        yc3 = yA-d*np.sin(theta2)

                        if math.isnan(xc2) == 1:
                            xC = xc1
                            yC = yc1
                        else:
                            xC = xc2
                            yC = yc2

                        # Output the bearing for publishing
                        bearing = np.array([theta2,xA,xmn,xmx,ymn,ymx,xc3,yc3],
                            dtype=np.float32)
                        rospy.logdebug("Bearing: %f,%f,%f,%f,%f,%f,%f,%f",theta2,xA,xmn,xmx,ymn,ymx,xc3,yc3)
        

        # If bearing does not exist, publish [0,0] instead
        if 'bearing' not in locals():
            bearing = np.array([0,0], dtype=np.float32)

        # Publish /bearing topic
        self.pub.publish(bearing)

if __name__ == '__main__':

    plt.ion()
    print "Looking for object..."
    laser_listener()
    rospy.spin()
