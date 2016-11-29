#!/usr/bin/env python

""" idvalve.py - Version 1.0 2016-10-12

    This program locates the center of the valve and updates the coordinates for moving
    arm to the correct location.

    Author: Jonathan Hodges, Virginia Tech

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

import rospy
# import rospkg
# import actionlib
# from actionlib_msgs.msg import *
# from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from sensor_msgs.msg import Image
import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import matplotlib.pyplot as plt
# from decimal import *
# import tf
# import math
# import random

class move2op():
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('idvalve', anonymous=True, log_level=rospy.DEBUG)

        # Enable shutdown in rospy (This is important so we cancel any move_base goals
        # when the node is killed)
        rospy.on_shutdown(self.shutdown) # Set rospy to execute a shutdown function when exiting

        # Store camera parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080

        # Set up ROS subscriber callback routines
        self.bridge = CvBridge()
        self.beari_sub = rospy.Subscriber("/bearing", numpy_msg(Floats), self.callback_bearing, queue_size=1)
        self.image_sub = rospy.Subscriber("/mybot/camera1/image_raw",Image,self.callback)
        self.image_pub = rospy.Publisher("image_topic_3",Image, queue_size=1)
        rospy.Subscriber("/valve", numpy_msg(Floats), self.callback_v_c, queue_size=1)

    def shutdown(self):
        rospy.sleep(1)

    def callback_bearing(self, bearing):
        ee_position = rospy.get_param('ee_position')
        self.xA = bearing.data[1]+0.461-ee_position[0]

    # callback_v_c is used to store the valve center topic into the class to be
    # referenced by the other callback routines.
    def callback_v_c(self, data):
        self.v_c = data.data

    # callback_wrench is used to store the wrench topic into the class to be
    # referenced by the other callback routines.
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            img_count = rospy.get_param('img_count')
            img_count = img_count + 1
        except:
            img_count = 0
        print "****************************************************"
        print "Image count: ", img_count
        #'/home/jonathan/idvalve_' + str(img_count) + '.png'
        
        cv2.imwrite('/home/jonathan/idvalve_rgb_%s_%s.png' % (str(int(1000*self.xA)), str(img_count)),cv_image)
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        lower = np.array([0,0,100], dtype = "uint8")
        upper = np.array([50,50,255], dtype = "uint8")
        mask = cv2.inRange(cv_image, lower, upper)
        output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        cimg = cv2.medianBlur(output,5)
        cimg = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)
        cv2.imwrite('/home/jonathan/idvalve_gra_%s_%s.png' % (str(int(1000*self.xA)), str(img_count)),cv_image_gray)
        rospy.set_param('img_count',img_count)
        #cv2.imshow('Gray',cimg)
        #cv2.waitKey(0)
        circles = cv2.HoughCircles(cimg, cv.CV_HOUGH_GRADIENT, 1, 1, param1=100, param2=10, minRadius=1, maxRadius=200)
        print np.shape(circles)
        if circles is not None:
            center_x = circles[0,:,0]
            center_y = circles[0,:,1]
            radius = circles[0,:,2]
            z = np.transpose(np.vstack((circles[0,:,0],circles[0,:,1])))
            term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
            flag = cv2.KMEANS_RANDOM_CENTERS
            k = 1
            ret2, labels, centers = cv2.kmeans(z, k, term_crit, 100, flag)

            radius_med = np.median(radius)
            print "Center location: ", centers[0]
            print "Median radius: ", radius_med
            val_loc = [(centers[0][0]),(centers[0][1])]
            """
            *************************************************************
            All of this code was done without color detection and may be useful in the future.
            *************************************************************
            center_x = circles[0,:,0]
            center_y = circles[0,:,1]
            radius = circles[0,:,2]
            for n in range(len(circles[0,:,1])):
                cv2.circle(cimg,(center_x[n],center_y[n]), radius[n],
                    (0,0,244), 2, cv2.CV_AA)

            z = np.transpose(np.vstack((circles[0,:,0],circles[0,:,1])))
            # Run K-means to determine centers and to which group each point belongs.
            term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
            flag = cv2.KMEANS_RANDOM_CENTERS
            k = 3
            ret2, labels, centers = cv2.kmeans(z, k, term_crit, 100, flag)
            hihi, bin_edges = np.histogram(labels, bins=3, range=[0,2])
            print hihi

            mn = np.min(hihi)
            ind = np.argmin(hihi)
            rospy.logdebug("All k-means centers: %s", " ".join(str(x) for x in centers))
            if mn < 50:
                rospy.logdebug("Optimum number of groups is 3")
                sz = np.shape(centers)
                centers2 = np.zeros([sz[0]-1,sz[1]])
                ct = 0
                for i in range(0,3):
                    if i != ind:
                        centers2[ct,:] = centers[i,:]
                        ct = ct + 1
                centers = centers2
            else:
                rospy.logdebug("Optimum number of groups is 2")
                k = 2
                ret2, labels, centers = cv2.kmeans(z, k, term_crit, 100, flag)
            centers = centers[centers[:,0].argsort()]
            print "Updated k-means centers: ", centers
            # Determine the center of the valve in the image
            cents = centers.copy()
            sz_full = np.shape(cimg)
            cents[:,0] = centers[:,0] - sz_full[1]/2
            cents[:,1] = centers[:,1] - sz_full[0]/2
            cents = np.multiply(cents,cents)

            cents2 = centers.copy()
            cents2[0,0] = centers[0,0]-centers[1,0]
            cents2[0,1] = centers[0,1]-centers[1,1]
            cents2 = np.multiply(cents2,cents2)
            dist2 = np.power(cents2[0,0]+cents2[0,1],0.5)/2
            if dist2 > 50:
                rospy.logdebug("Optimum number of groups is 2")
                dist = np.zeros([k,1])
                for i in range(0,2):
                    dist[i] = np.power(cents[i,0]+cents[i,1],0.5)/2
                dist_loc = np.argmin(dist)
                rospy.logdebug("Minimum distance from center: %s", str(dist_loc))
                if len(dist) > 1:
                   dist[dist_loc] = 99999999
                   dist_loc = np.argmin(dist)
                else:
                   dist_loc = dist_loc
                rospy.logdebug("Minimum distance from center 2nd place: %s", str(dist_loc))
                val_loc = [centers[dist_loc,0],centers[dist_loc,1]]
            else:
                rospy.logdebug("Optimum number of groups is 1")
                k = 1
                ret2, labels, centers = cv2.kmeans(z, k, term_crit, 100, flag)
                val_loc = [centers[0,0],centers[0,1]]
            rospy.logdebug("Valve location in px: %s %s", str(val_loc[0]), str(val_loc[1]))
            #cv2.imshow('All Circles',cimg)
            #cv2.waitKey(0)
            cv2.imwrite('/home/jonathan/valveID_0_circs.png',cimg)
            rospy.logdebug("Valve in pixels: %s", " ".join(str(x) for x in val_loc))
            """

            # Find camera dimensions wrt the base coordinate system
            camera_y_mx = self.xA*np.arctan(self.camera_fov_h/2)
            camera_y_mn = -1*self.xA*np.arctan(self.camera_fov_h/2)
            camera_z_mx = self.xA*np.arctan(self.camera_fov_v/2)
            camera_z_mn = -1*self.xA*np.arctan(self.camera_fov_v/2)
            rospy.logdebug("Camera ymn/ymx: %s %s", str(camera_y_mn), str(camera_y_mx))
            rospy.logdebug("Camera zmn/zmx: %s %s", str(camera_z_mn), str(camera_z_mx))

            # Convert the valve pixel loacation
            valve_y = (1-val_loc[0]/1920)*(camera_y_mx-camera_y_mn)+camera_y_mn
            valve_z = (1-val_loc[1]/1080)*(camera_z_mx-camera_z_mn)+camera_z_mn

            self.valve_id = np.array([self.xA, valve_y, valve_z],dtype=np.float32)
            rospy.logdebug("Valve in m: %s", " ".join(str(x) for x in self.valve_id))
            rospy.set_param('valve_ID',[float(self.valve_id[0]), float(self.valve_id[1]), float(self.valve_id[2])])
            err = np.power(valve_y*valve_y+valve_z*valve_z,0.5)
            if err < 0.01:
                # Valve is centered no other action required
                rospy.set_param('smach_state','valveCenter')
                rospy.logdebug("*****************************************************")
                rospy.logdebug("We are centered! error %s", str(err))
            else:
                # Valve not centered, publish new move parameters
                rospy.set_param('smach_state','valveOffCenter')
                valve_ID_ready_pos = rospy.get_param('valve')
                valve_ID_ready_pos[0] = self.xA
                valve_ID_ready_pos[1] = self.valve_id[1]
                valve_ID_ready_pos[2] = self.valve_id[2]

                #rospy.set_param('ee_position', [float(valve_ID_ready_pos[0]-0.5),
                #                                float(valve_ID_ready_pos[1]),
                #                                float(valve_ID_ready_pos[2])])

                rospy.set_param('valve', [float(valve_ID_ready_pos[0]),
                                          float(valve_ID_ready_pos[1]),
                                          float(valve_ID_ready_pos[2])])
        else:
            rospy.set_param('smach_state','valveNotFound')

        rospy.signal_shutdown('Ending node.')

if __name__ == '__main__':
    try:
        move2op()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("idvalve finished.")

