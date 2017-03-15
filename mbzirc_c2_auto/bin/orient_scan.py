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
from nav_msgs.msg import Odometry
import StringIO
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import tf

class orient_scan():
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
        self.physical_robot = rospy.get_param('physical_robot')
        if fake_lidar == 'False':
            if self.physical_robot:
                rospy.Subscriber("/scan/close_range",sensor_msgs.msg.LaserScan,self.cb_scan,
                    queue_size=1)
            else:
                rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,self.cb_scan,
                    queue_size=1)
        rospy.Subscriber('/odometry/filtered',Odometry, self.cb_odom)
        self.pub = rospy.Publisher("/bearing",numpy_msg(Floats), queue_size=1)
        self.bridge = CvBridge()
        if fake_lidar == 'True':
            while not rospy.is_shutdown():
                bearing = np.array([0,0.69,0,0,0,0,0,0],dtype=np.float32)
                self.pub.publish(bearing)
                rospy.sleep(0.1)
        # Set parameters
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
        self.scan_dist_thresh = 0.1
        self.plot_data = False
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
        """callback (/scan) is used to segment a laser scan into continuous
        segments and identify if the robot is normal to the object scan
        """

        # Set max/min angle and increment
        scan_min = data.angle_min
        scan_max = data.angle_max
        scan_inc = data.angle_increment

        # Build angle array
        if self.physical_robot:
            y = np.arange(scan_min,scan_max+scan_inc*0.1,scan_inc)#-1.57
        else:
            y = np.arange(scan_min,scan_max+0.01*scan_inc,scan_inc)

        # Compute sine and cosine of each LIDAR angle
        ysin = np.sin(y)
        ycos = np.cos(y)

        # Apply a median filter to the LIDAR scan
        x = sg.medfilt(data.ranges,1)-0.25

        # Decompose range measurement into local x,y coordinates (x forward, y to left)
        y_coord = x*ysin
        x_coord = x*ycos

        # Calculate difference between each consecutive scan
        x_diff = np.power(np.diff(x_coord),2)
        y_diff = np.power(np.diff(y_coord),2)
        dist = np.power(x_diff+y_diff,0.5)

        # Segment the scan into objects based on a distance greater than the threshold
        x2 = np.array(np.split(x, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))
        y2 = np.array(np.split(y, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))
        dist2 = np.array(np.split(dist, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))
        x_coord2 = np.array(np.split(x_coord, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))
        y_coord2 = np.array(np.split(y_coord, np.argwhere(dist > self.scan_dist_thresh).flatten()[1:]))

        # Loop through each segmented object
        [x_coord_glo,y_coord_glo] = np.dot(self.R,[x_coord2,y_coord2])
        x_coord_glo = self.x0+x_coord_glo
        y_coord_glo = self.y0+y_coord_glo

        # Loop through each segmented object to check if it is the appropriate size
        for i in range(len(y2)):
            ylen = len(y2[i])-0
            if ylen > self.ylen_lim:
                # Calculate how long the scan is
                dist2_sum = np.sum(dist2[i][1:ylen-1])
                y_pt = np.median(y_coord_glo[i])
                x_pt = np.median(x_coord_glo[i])
                # Check if the scan is just right
                if dist2_sum > self.dist_min and dist2_sum < self.dist_max:
                    #mxmx = max(iii for iii in x_coord2[i] if iii < 30)
                    #if mxmx > 0.5:
                    if y_pt < self.arena_ypos and y_pt > self.arena_yneg and x_pt < self.arena_xpos and x_pt > self.arena_xneg:
                        # Find median angle of scan
                        ang = np.median(y2[i])
                        idx = np.argwhere(y2[i] == ang)
                        if ang > self.ang_min and ang < self.ang_max:
                            # If no median exists because an even number of entries, ignore
                            # the first entry to make it an odd number of entries
                            if len(idx) == 0:
                                ang = np.median(y2[i][1:ylen])
                                idx = np.argwhere(y2[i] == ang)

                            # Store minimum and maximum extents of scan
                            xmn = min(x_coord2[i][1:ylen])
                            xmx = max(x_coord2[i][1:ylen])
                            ymn = min(y_coord2[i][1:ylen])
                            ymx = max(y_coord2[i][1:ylen])

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

        # If bearing does not exist, publish [0,0] instead
        if 'bearing' not in locals():
            bearing = np.array([0,0], dtype=np.float32)

        # Publish /bearing topic
        self.pub.publish(bearing)

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
                                x_good = x_pt; y_good = y_pt;
                                plt.plot(y_coord_glo[i][1:ylen],x_coord_glo[i][1:ylen],'r-', linewidth=2.0)
            # Show arena bounds
            plt.plot([self.arena_ypos,self.arena_ypos],[self.arena_xneg,self.arena_xpos],'k-',linewidth=4.0)
            plt.plot([self.arena_yneg,self.arena_yneg],[self.arena_xneg,self.arena_xpos],'k-',linewidth=4.0)
            plt.plot([self.arena_yneg,self.arena_ypos],[self.arena_xneg,self.arena_xneg],'k-',linewidth=4.0)
            plt.plot([self.arena_yneg,self.arena_ypos],[self.arena_xpos,self.arena_xpos],'k-',linewidth=4.0)
            # Show UGV
            plt.plot(self.y0,self.x0,'mo',markersize=10.0)
            plt.xlim([y_good-10,y_good+10])
            plt.ylim([x_good-10,x_good+10])
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
            self.image_output.publish(bridge.cv2_to_imgmsg(im, "bgr8"))
            plt.close()
        pass

if __name__ == '__main__':

    print "Looking for object..."
    orient_scan()
    rospy.spin()
