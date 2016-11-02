#!/usr/bin/env python 

""" orient_scan.py - Version 1.0 2016-10-12

    This software uses a LIDAR scan to identify a box.
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

class laser_listener():
    # Initialize the class
    def __init__(self):
        # Name this node, it must be unique
        rospy.init_node('orient_scan', anonymous=True)

        # Set up ROS subscriber callback routines
        rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,self.callback, queue_size=1)

        # Initialize counter for plot debugging (to flip y-axis for better viewing)
        self.pt = 0

    # callback (/scan) is used to segment a laser scan into continuous segments
    # and identify if the robot is normal to the object scan
    def callback(self, data):
        # Set ros refresh rate
        rate = rospy.Rate(10)

        # Initialize parameters
        plot_flag = 0
        debug_flag = 0
        thresh = 0.1

        # Establish LIDAR test angles. This must match the parameters of the LIDAR
        # in gazebo.
        y = np.arange(-1.66,1.6525,0.0029146999)

        # Compute sine and cosine of each LIDAR angle
        ysin = np.sin(y)
        ycos = np.cos(y)

        # If we want to plot the LIDAR scan, open the plot environment
        if plot_flag == 1:
            plt.cla()

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

                # Check if the scan is too short
                if dist2_sum < 0.25:
                #plt.plot(x2[i][1:xlen],y2[i][1:xlen],'k-')
                #plt.plot(x2[i],y_coord2[i],'ks')
                    hihi = 1
                else:

                    # Check if the scan is too long
                    if dist2_sum > 5:
                    #plt.plot(x2[i][1:xlen],y2[i][1:xlen],'b-')
                    #plt.plot(x2[i],y_coord2[i],'bs')
                        hihi = 1
                    
                    # Check if the scan is just right
                    else:
                        
                        # Plot the scan if desired
                        if plot_flag == 1:
                            plt.plot(y_coord2[i][1:xlen],x_coord2[i][1:xlen],'r-')

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
                        xA = x_coord2[i][idx]
                        yA = y_coord2[i][idx]

                        # Store physical (x,y) coordinates in local system of minimum
                        # range of scan
                        idx2 = np.argwhere(x_coord2[i] == xmn)
                        xB = x_coord2[i][idx2]
                        yB = y_coord2[i][idx2]

                        # Determine which way the robot needs to rotate based on the
                        # physical orientations of the median and minimum scans
                        if xA > xB:
                            if yA > yB:
                                if debug_flag == 1:
                                    print "xA > xB, yA > yB"
                                rot = -1
                            else:
                                if debug_flag == 1:
                                    print "xA > xB, yA < yB"
                                rot = 1
                        else:
                            if yA > yB:
                                if debug_flag == 1:
                                    print "xA < xB, yA > yB"
                                rot = -1
                            else:
                                if debug_flag == 1:
                                    print "xA < xB, yA < yB"
                                rot = 1

                        # Output the bearing for publishing
                        bearing = np.array([rot*0.2,xA,yA,xB,yB,xmn,xmx,ymn,ymx], dtype=np.float32)
                        if debug_flag == 1:
                            print bearing
        
        # If bearing does not exist, publish [0,0] instead
        if 'bearing' in locals():
            hihi = 1
        else:
            bearing = np.array([0,0], dtype=np.float32)

        # Publish /bearing topic
        pub = rospy.Publisher("/bearing",numpy_msg(Floats), queue_size=1)
        pub.publish(bearing)

        # If plotting the scan, make it pretty
        if plot_flag == 1:
            if self.pt == 0:
                plt.gca().invert_xaxis()
                self.pt = self.pt +1
            plt.xlabel('y Distance [m]')
            plt.ylabel('x Distance [m]')
            plt.title('Laser Scan')
            plt.draw()
        pass

if __name__ == '__main__':
    
    plt.ion()
    print "Looking for object..."
    laser_listener()
    rospy.spin()
