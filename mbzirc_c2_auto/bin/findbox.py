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
from nav_msgs.msg import Odometry
import StringIO
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import urllib, base64
import os
import sys
import math
import tf
import time

class findbox():
    def __init__(self):
        rospy.init_node('findbox', anonymous=True)
        self.physical_robot = rospy.get_param('physical_robot')
        if self.physical_robot:
            rospy.Subscriber("/scan/long_range",sensor_msgs.msg.LaserScan,self.cb_scan, queue_size=1)
        else:
            rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,self.cb_scan, queue_size=1)
        rospy.Subscriber('/odometry/filtered',Odometry, self.cb_odom)
        self.bearing_pub = rospy.Publisher("/detection",numpy_msg(Floats), queue_size=1)
        self.bridge = CvBridge()
        self.dist_min = 0.25
        self.dist_max = 3
        self.ylen_lim = 4
        self.ang_min = -1.5
        self.ang_max = 1.5

        self.arena_xpos = rospy.get_param('arena_xpos')
        self.arena_xneg = rospy.get_param('arena_xneg')
        self.arena_ypos = rospy.get_param('arena_ypos')
        self.arena_yneg = rospy.get_param('arena_yneg')

        self.rate = rospy.Rate(10)
        self.scan_dist_thresh = 0.1  # Distance threshold to split obj into 2 obj.
        self.plot_data = True
        self.image_output = rospy.Publisher("/output/keyevent_image",Image, 
            queue_size=1)

    def cb_odom(self, data):
        self.odom = data.pose.pose
        self.x0 = self.odom.position.x
        self.y0 = self.odom.position.y
        X0 = self.odom.orientation.x
        Y0 = self.odom.orientation.y
        Z0 = self.odom.orientation.z
        W0 = self.odom.orientation.w
        [roll,pitch,yaw] = tf.transformations.euler_from_quaternion([X0,Y0,Z0,W0])
        self.R = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])

    def cb_scan(self, data):
        """
        This callback runs each time a LIDAR scan is obtained from
        the /scan topic in ROS. Returns a topic /detection. If no
        object is found, /detection = [0,0]. If an object is found,
        /detection = [angle,distance] to median of scan.
        """

        # Set max/min angle and increment
        scan_min = data.angle_min
        scan_max = data.angle_max
        scan_inc = data.angle_increment

        # Build angle array
        if self.physical_robot:
            y = np.arange(scan_min,scan_max,scan_inc)-1.57
        else:
            y = np.arange(scan_min,scan_max+0.01*scan_inc,scan_inc)

        # Pre-compute trig functions of angles
        ysin = np.sin(y)
        ycos = np.cos(y)

        # Apply a median filter to the range scans
        x = sg.medfilt(data.ranges,1)

        # Calculate the difference between consecutive range values
        x_diff1 = np.power(np.diff(x),2)

        # Convert range and bearing measurement to cartesian coordinates
        y_coord = x*ysin
        x_coord = x*ycos

        # Compute difference between consecutive values in cartesian coordinates
        y_diff = np.power(np.diff(y_coord),2)
        x_diff = np.power(np.diff(x_coord),2)

        # Compute physical distance between measurements
        dist = np.power(x_diff+y_diff,0.5)

        # Segment the LIDAR scan based on physical distance between measurements
        x2 = np.array(np.split(x, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))
        y2 = np.array(np.split(y, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))
        dist2 = np.array(np.split(dist, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))
        x_coord2 = np.array(np.split(x_coord, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))
        y_coord2 = np.array(np.split(y_coord, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))

        # Loop through each segmented object
        [x_coord_glo,y_coord_glo] = np.dot(self.R,[x_coord2,y_coord2])
        x_coord_glo = self.x0+x_coord_glo
        y_coord_glo = self.y0+y_coord_glo
        
        for i in range(len(y2)):
            # Check if there are at least 4 points in an object (reduces noise)
            ylen = len(y2[i])-0
            if ylen > self.ylen_lim:
                # Calculate distance of this object
                dist2_sum = np.sum(dist2[i][1:ylen-1])
                y_pt = np.median(y_coord_glo[i])
                x_pt = np.median(x_coord_glo[i])
                # Check if this object is too small
                if dist2_sum > self.dist_min and dist2_sum < self.dist_max:
                    if y_pt < self.arena_ypos and y_pt > self.arena_yneg and x_pt < self.arena_xpos and x_pt > self.arena_xneg:
                        ang = np.median(y2[i])
                        dis = np.median(x2[i])
                        mn = min(x2[i][1:ylen])
                        mx = max(x2[i][1:ylen])
                        if ang > self.ang_min and ang < self.ang_max:
                            bearing = np.array([ang,dis], dtype=np.float32)
                            #bearing = np.array([y_pt,x_pt], dtype=np.float32)

        # Check if bearing exists. Store [0,0] if no object was found
        if 'bearing' not in locals():
            bearing = np.array([0,0], dtype=np.float32)

        # Publish bearing to ROS on topic /detection
        self.bearing_pub.publish(bearing)

        # If we want to plot the LIDAR scan, open the plot environment
        if self.plot_data:
            plt.figure(1)
            for i in range(len(y2)):
                ylen = len(y2[i])-0
                if ylen > self.ylen_lim:
                    dist2_sum = np.sum(dist2[i][1:ylen-1])
                    y_pt = np.median(y_coord_glo[i])
                    x_pt = np.median(x_coord_glo[i])
                    if y_pt > self.arena_ypos or y_pt < self.arena_yneg or x_pt > self.arena_xpos or x_pt < self.arena_xneg:
                        print "x_mn, x_mx, x_pt: ", self.arena_xneg, self.arena_xpos, x_pt
                        print "y_mn, y_mx, y_pt: ", self.arena_yneg, self.arena_ypos, y_pt
                        #plt.plot(y_pt,x_pt,'co',markersize=10.0)
                        plt.plot(y_coord_glo[i][1:ylen],x_coord_glo[i][1:ylen],'c-',linewidth=2.0)
                    else:
                        if dist2_sum < self.dist_min:
                            plt.plot(y_coord_glo[i][1:ylen],x_coord_glo[i][1:ylen],'k-',linewidth=2.0)
                        else:
                            if dist2_sum > self.dist_max:
                                plt.plot(y_coord_glo[i][1:ylen],x_coord_glo[i][1:ylen],'b-', linewidth=2.0)
                            else:
                                plt.plot(y_coord_glo[i][1:ylen],x_coord_glo[i][1:ylen],'r-', linewidth=2.0)
            # Show arena bounds
            plt.plot([self.arena_ypos,self.arena_ypos],[self.arena_xneg,self.arena_xpos],'k-',linewidth=4.0)
            plt.plot([self.arena_yneg,self.arena_yneg],[self.arena_xneg,self.arena_xpos],'k-',linewidth=4.0)
            plt.plot([self.arena_yneg,self.arena_ypos],[self.arena_xneg,self.arena_xneg],'k-',linewidth=4.0)
            plt.plot([self.arena_yneg,self.arena_ypos],[self.arena_xpos,self.arena_xpos],'k-',linewidth=4.0)
            # Show UGV
            plt.plot(self.y0,self.x0,'mo',markersize=10.0)
            plt.xlim([-40,40])
            plt.ylim([-10,30])
            plt.gca().invert_xaxis()
            plt.xlabel('Left of robot [m] ')
            plt.ylabel('Front of robot [m]')
            plt.title('Laser Scan')
            imgdata = StringIO.StringIO()
            plt.savefig(imgdata, format='png')
            imgdata.seek(0)
            img_array = np.asarray(bytearray(imgdata.read()), dtype=np.uint8)
            im = cv2.imdecode(img_array, 1)
            self.image_output.publish(self.bridge.cv2_to_imgmsg(im, "bgr8"))
            plt.close()
        pass

if __name__ == '__main__':
    rospy.loginfo('Looking for object...')
    findbox()
    rospy.spin()
