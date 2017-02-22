#!/usr/bin/env python

"""orient_scan.py - Version 1.0 2016-10-12
Author: Jonathan Hodges

This code searches a 2-D LIDAR scan for an object within a minimum and maximum
length bound. The LIDAR scan is segmented based on null returns and large
deviations between points.

Subscribers:
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
import math

class laser_listener():
    """MBZIRC Challenge 2 orient_scan

    This code searches a 2-D LIDAR scan for an object within a minimum and
    maximum length bound. The LIDAR scan is segmented based on null returns and
    large deviations between points.

    Attributes:
        self.pub: Publisher for bearing topic

    Subscribers:
        /scan: 2-D LIDAR scan from ROS

    Publishers:
        /bearing: array containing
            [angle,x_med,xmn,xmx,ymn,ymx,target_x,target_y]
"""

    def __init__(self):
        """This initializes the publishers and subscribers in the class.
        """
        # Name this node, it must be unique
        rospy.init_node('orient_scan', anonymous=True)
        try:
            fake_lidar = rospy.get_param('fake_lidar')
        except:
            fake_lidar = 'False'
        # Set up ROS subscriber callback routines
        if fake_lidar == 'False':
            rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,self.callback,
                queue_size=1)
        self.pub = rospy.Publisher("/bearing",numpy_msg(Floats), queue_size=1)
        if fake_lidar == 'True':
            while not rospy.is_shutdown():
                bearing = np.array([0,0.69,0,0,0,0,0,0],dtype=np.float32)
                self.pub.publish(bearing)
                rospy.sleep(0.1)

    def callback(self, data):
        """callback (/scan) is used to segment a laser scan into continuous
        segments and identify if the robot is normal to the object scan
        """
        # Set ros refresh rate
        rate = rospy.Rate(10)

        # Initialize parameters
        thresh = 0.1

        # Set max/min angle and increment
        scan_min = data.angle_min
        scan_max = data.angle_max
        scan_inc = data.angle_increment

        # Build angle array
        y = np.arange(scan_min,scan_max,scan_inc)#-1.57

        # Compute sine and cosine of each LIDAR angle
        ysin = np.sin(y)
        ycos = np.cos(y)

        # Apply a median filter to the LIDAR scan
        x = sg.medfilt(data.ranges,1)

        # Decompose range measurement into local x,y coordinates (x forward, y to left)
        y_coord = x*ysin
        x_coord = x*ycos

        # Calculate difference between each consecutive scan
        x_diff = np.power(np.diff(x_coord),2)
        y_diff = np.power(np.diff(y_coord),2)
        dist = np.power(x_diff+y_diff,0.5)

        # Segment the scan into objects based on a distance greater than the threshold
        x2 = np.array(np.split(x, np.argwhere(dist > thresh).flatten()[1:]))
        y2 = np.array(np.split(y, np.argwhere(dist > thresh).flatten()[1:]))
        dist2 = np.array(np.split(dist, np.argwhere(dist > thresh).flatten()[1:]))
        x_coord2 = np.array(np.split(x_coord, np.argwhere(dist > thresh).flatten()[1:]))
        y_coord2 = np.array(np.split(y_coord, np.argwhere(dist > thresh).flatten()[1:]))

        # Loop through each segmented object to check if it is the appropriate size
        for i in range(len(x2)):
            xlen = len(x2[i])-0

            # Only consider scans with at least 4 entries (cuts down on noise)
            if xlen > 4:

                # Calculate how long the scan is
                dist2_sum = np.sum(dist2[i][1:xlen-1])

                # Check if the scan is just right
                if dist2_sum > 0.25 and dist2_sum < 5:
                    mxmx = max(iii for iii in x_coord2[i] if iii < 30)
                    if mxmx > 0.5:
                        # Find median angle of scan
                        ang = np.median(y2[i])
                        idx = np.argwhere(y2[i] == ang)

                        # If no median exists because an even number of entries, ignore
                        # the first entry to make it an odd number of entries
                        if len(idx) == 0:
                            ang = np.median(y2[i][1:xlen])
                            idx = np.argwhere(y2[i] == ang)

                        # Store minimum and maximum extents of scan
                        xmn = min(x_coord2[i][1:xlen])
                        xmx = max(x_coord2[i][1:xlen])
                        ymn = min(y_coord2[i][1:xlen])
                        ymx = max(y_coord2[i][1:xlen])

                        # Store physical (x,y) coordinates in local system of median
                        # angle of scan
                        xA_thresh = x_coord2[i]
                        xA = np.median(xA_thresh[xA_thresh > 0.5])
                        yA = y_coord2[i][idx]

                        # Store physical (x,y) coordinates in local system of minimum
                        # range of scan
                        #print "x_coord2: ", x_coord2[i]
                        idx2 = np.argwhere(x_coord2[i] == xmn)
                        xB = x_coord2[i][idx2]
                        yB = y_coord2[i][idx2]

                        # Determine which way the robot needs to rotate based on the
                        # physical orientations of the median and minimum scans
                        m = (yB-yA)/(xB-xA)
                        m2 = 1/m
                        theta2 = math.atan(-m2/1)
                        roboX = rospy.get_param("/currentRobotX")
                        roboY = rospy.get_param("/currentRobotY")
                        arenaPnt1 = rospy.get_param("/arenaPnt1")
                        arenaPnt2 = rospy.get_param("/arenaPnt2")
                        deadZone1 = rospy.get_param("/deadZone1")
                        deadZone2 = rospy.get_param("/deadZone2")
                        roboR = rospy.get_param("/currentRobotR")
                        detectX = roboX + (xA * math.cos(theta2 + roboR))
                        detectY = roboY - (xA * math.sin(theta2 + roboR))

                        if detectX > arenaPnt1[0] and detectX < arenaPnt2[0] and detectY < arenaPnt1[1] and detectY > arenaPnt2[1] and not (detectX > deadZone1[0] and detectX < deadZone2[0] and detectY < deadZone1[1] and detectY > deadZone2[1]):
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
                            bearing = np.array([theta2,xA,xmn,xmx,ymn,ymx,xc3,yc3], dtype=np.float32)

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
