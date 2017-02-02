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
import math
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
        self.ct5 = 0
        self.lim_type = 0
        self.xA = 0.380
        self.save_result = True
        self.preview_flag = True           # Should we preview images
        self.preview_result = True
        self.indir = '/home/jonathan/'
        self.lim_adjust = 140

        # Set up ROS subscriber callback routines
        self.bridge = CvBridge()
        self.beari_sub = rospy.Subscriber("/bearing", numpy_msg(Floats), self.callback_bearing, queue_size=1)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.image_pub = rospy.Publisher("image_topic_3",Image, queue_size=1)
        self.image_circ_pub = rospy.Publisher("/output/valve_center",Image, queue_size=1)
        rospy.Subscriber("/valve", numpy_msg(Floats), self.callback_v_c, queue_size=1)
        self.image_output = rospy.Publisher("/output/keyevent_image",Image, queue_size=1)
    def shutdown(self):
        rospy.sleep(1)

    def callback_bearing(self, bearing):
        ee_position = rospy.get_param('ee_position')
        if bearing.data[1] != 0:
            self.xA = bearing.data[1]+0.4-ee_position[0]
        else:
            if self.xA == 0:
                self.xA = 0.67+0.4-ee_position[0]

    # callback_v_c is used to store the valve center topic into the class to be
    # referenced by the other callback routines.
    def callback_v_c(self, data):
        self.v_c = data.data

    # callback_wrench is used to store the wrench topic into the class to be
    # referenced by the other callback routines.
    def callback(self, data):

        def stretchlim(img):
            """This subroutine computes the limits to be used in the brightness
            and contrast adjustment routine. The lim_type denotes whether to
            fix the upper limit based on the highest one percent of the data or
            not. 0 = Dynamic fix, 1 = Fixed at an intensity of 255.
            """
            # Determine size of image in pixels
            sz = np.shape(img)
            num_of_px = sz[0]*sz[1]
            # Determine one percent of total pixels (for use in image adjust code)
            one_perc = math.floor(num_of_px*0.01)
            lims = np.zeros((sz[2],2))
            # Compute lower/upper 1% threshold for each channel
            for i in range(0,sz[2]):
                hist,bins = np.histogram(img[:,:,i].ravel(),255,[0,255])
                val = 0; j = 0;
                while val < one_perc:
                    val = val+hist[j]
                    j = j +1
                lims[i,0] = j
                if self.lim_type == 0:
                    val = 0; j = 0;
                    while val < one_perc:
                        val = val+hist[254-j]
                        j = j + 1
                    lims[i,1] = 254-j-self.lim_adjust
                if self.lim_type == 1:
                    lims[i,1] = 255
            return lims

        # This subroutine adjusts the intensities in each channel of the RGB image
        # using the limits supplied by stretchlim. Returns the adjusted image.
        def imadjust(img,lims):
            img2 = np.copy(img)
            sz = np.shape(img2)
            # Loop through each channel in the image
            for i in range(0,sz[2]):
                I2 = img2[:,:,i]
                # Set intensities above and below threshold to caps to prevent
                # overflow in 8bit numbers.
                I2[I2 > lims[i,1]] = lims[i,1]
                I2[I2 < lims[i,0]] = lims[i,0]
                # Scale the intensity in the channel
                img2[:,:,i] = (I2-lims[i,0])/(lims[i,1]-lims[i,0])*255
            return img2

        # This subroutine removes the background from the RGB image by increasing
        # the intensity in each channel of the image by (1/2) of the maximum value
        # within that channel. Returns the RGB image after background removal.
        def back_ground_remove(I,I_old):
            """This subroutine removes the background from the RGB image by
            increasing the intensity in each channel of the image by (1/2) of
            the maximum value within that channel. Returns the RGB image after
            background removal.
            """
            # Determine size of image in pixels
            sz = np.shape(I)
            # Initialize intensity array
            i = np.zeros((sz[2],1))
            # Initialize updated intensity matrix
            I3 = I.copy()
            # Loop through each channel of the image
            for j in range(0,sz[2]):
                # Caculate the intensity histogram of one channel
                hist,bins = np.histogram(I[:,:,j].ravel(),255,[0,255])
                I2 = I[:,:,j].copy()
                # Find the most common bin in the histogram
                i[j] = np.argmax(hist)
                # Fix overflow problems by setting values greater than the
                # modifier such that they will be maxed after addition
                I2[I2 > 255-i[j]*0.5] = 255-i[j]*0.5
                # Add the intensity modifier
                I2 = I2+0.5*i[j]
                # Update intensity matrix
                I3[:,:,j] = I2
            I4 = cv2.cvtColor(I3, cv2.COLOR_BGR2GRAY)
            I_old[I4 == 255,:] = 255
            #I_old[I3 == 255] = 255
            return I_old

        def quantize_image(img):
            """This subroutine quantizes an image into 3 kmeans color groups
            """
            sz = np.shape(img)
            z = np.float32(img.copy())
            z = z.reshape((sz[0]*sz[1],1))
            print np.shape(z)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            ret,labels,centers = cv2.kmeans(z,3,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
            mean_sort = np.argsort(centers.flatten())
            z = z.reshape((sz[0],sz[1]))
            labels = labels.reshape((sz[0],sz[1]))
            A = img.copy()
            B = img.copy()
            A[labels != mean_sort[0]] = 0
            B[labels != mean_sort[1]] = 0
            return A, B

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
        #'/home/administrator/idvalve_' + str(img_count) + '.png'
        
        cv2.imwrite('%sidvalve_rgb_%s_%s.png' % (self.indir, str(int(1000*self.xA)), str(img_count)),cv_image)

        cimg = cv2.medianBlur(cv_image,5)
        lims = stretchlim(cimg)
        print lims
        img_invert = 255-cimg.copy()
        if self.preview_flag:
            cv2.imshow('img_invert',img_invert)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        img_adj = imadjust(img_invert.copy(),lims)
        if self.preview_flag:
            cv2.imshow('img_adj',img_adj)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        img_remove = back_ground_remove(img_adj.copy(),img_invert.copy())
        if self.preview_flag:
            cv2.imshow('img_remove',img_remove)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        img_remove_gray = cv2.cvtColor(img_remove, cv2.COLOR_BGR2GRAY)
        """
        img_hsv = cv2.cvtColor(cimg, cv2.COLOR_BGR2HSV)
        #print np.max(img_hsv[:,:,0]), np.max(img_hsv[:,:,1]), np.max(img_hsv[:,:,2])

        sz = np.shape(cimg)
        for i in range(0,sz[0]):
            for j in range(0,sz[1]):
                if img_hsv[i,j,1] < 50:
                    img_hsv[i,j,:] = 0
                #img_remove_gray[i,j] = np.max(img_remove[i,j,:])
        img_remove_gray = img_hsv[:,:,2]
        """
        if self.preview_flag:
            cv2.imshow('img_remove_gray',img_remove_gray)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        [A,B] = quantize_image(img_remove_gray.copy())
        if self.preview_flag:
            cv2.imshow('quantize_A',A)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        if self.preview_flag:
            cv2.imshow('quantize_B',B)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


        #z = np.transpose(np.vstack((circles[0,:,0],circles[0,:,1])))
        # Run K-means to det. centers and to which group each point belongs

        """
        term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
        flag = cv2.KMEANS_RANDOM_CENTERS
        z = np.float32(img_remove_gray.reshape((-1,1)))
        ret2, labels, centers = cv2.kmeans(z, 3, term_crit, 10, flag)
        centers = np.uint8(centers)
        res = centers[labels.flatten()]
        res2 = res.reshape((img_remove_gray.shape))

        cv2.imshow('res2',res2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        """

                


        #cv2.imshow('Image before circles',img_remove_gray)
        #cv2.waitKey(0)
        #cv2.imwrite('/home/administrator/idvalve_gra_%s_%s.png' % (str(int(1000*self.xA)), str(img_count)),cv_image_gray)
        rospy.set_param('img_count',img_count)
        #cv2.imshow('Gray',cimg)
        #cv2.waitKey(0)
        #self.xA = 0.455
        rad_px_old =int(130*340/(1000*self.xA))
        rad_px = int(-0.9143*self.xA*1000+471.31+20)
        print "Estimated pixel radius: ", rad_px
        print "Estimated old pixel radius: ", rad_px_old
        term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
        flag = cv2.KMEANS_RANDOM_CENTERS
        A_circles = cv2.HoughCircles(A, cv.CV_HOUGH_GRADIENT, 1, 1, param1=100, param2=40,
            minRadius=rad_px-50, maxRadius=rad_px+300)
        A_z = np.transpose(np.vstack((A_circles[0,:,0],A_circles[0,:,1])))
        ret2, labels, A_centers = cv2.kmeans(A_z, 1, term_crit, 100, flag)

        B_circles = cv2.HoughCircles(B, cv.CV_HOUGH_GRADIENT, 1, 1, param1=100, param2=40,
            minRadius=rad_px-50, maxRadius=rad_px+300)
        B_z = np.transpose(np.vstack((B_circles[0,:,0],B_circles[0,:,1])))
        ret2, labels, B_centers = cv2.kmeans(B_z, 1, term_crit, 100, flag)
        A_centers = np.floor(A_centers)
        B_centers = np.floor(B_centers)
        print A_centers, B_centers
        quant_vote = np.empty([2,1])
        for i in range(-100,100):
            for j in range(-100,100):
                try:
                    if A[A_centers[0][0]+i,A_centers[0][1]+j] != 0:
                        quant_vote[0] = quant_vote[0]+1
                    if B[B_centers[0][0]+i,B_centers[0][1]+j] != 0:
                        quant_vote[1] = quant_vote[1]+1
                except:
                    pass
        if quant_vote[0] > quant_vote[1]:
            val_loc = A_centers
            val_rad = A_circles[0,:,2]
            wrench_loc = B_centers
            wrench_rad = B_circles[0,:,2]
            print "Quantize A wins."
        else:
            val_loc = B_centers
            val_rad = B_circles[0,:,2]
            wrench_loc = A_centers
            wrench_rad = B_circles[0:,2]
            print "Quantize B wins."

        centers = val_loc
        radius_med = np.median(val_rad)
        ind = 0
        if A_centers is not None:
            """
        circles = cv2.HoughCircles(img_remove_gray, cv.CV_HOUGH_GRADIENT, 1, 1, param1=60, param2=30
, minRadius=rad_px-50, maxRadius=rad_px+50)
        #cv2.imshow('Image before circles',cimg)
        term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
        flag = cv2.KMEANS_RANDOM_CENTERS
        k = 0
        ret2 = 999999
        while ret2 > 500000:
            k = k+1
            z = np.transpose(np.vstack((circles[0,:,0],circles[0,:,1])))
            ret2, labels, centers = cv2.kmeans(z, k, term_crit, 100, flag)
            print "Ret2: ", ret2

        if circles is not None:
            img_hou_all = cimg.copy()
            center_x = circles[0,:,0]
            center_y = circles[0,:,1]
            radius = circles[0,:,2]
            for n in range(len(circles[0,:,1])):
                cv2.circle(img_hou_all,(center_x[n],center_y[n]), radius[n],
                    (255,255,255), 2, cv2.CV_AA)
            if self.preview_flag:
                cv2.imshow('img',img_hou_all)
                cv2.waitKey(0)

            z = np.transpose(np.vstack((circles[0,:,0],circles[0,:,1])))
            term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
            flag = cv2.KMEANS_RANDOM_CENTERS
            #k = 1
            ret2, labels, centers = cv2.kmeans(z, k, term_crit, 100, flag)
            cents = np.power(centers[:]-[1920/2,1080/2],2)
            dist = np.sum(cents,axis=1)
            print "Dist: ", dist
            ind = np.argmin(dist)
            dist[ind] = 99999999
            print "Dist: ", dist
            ind = np.argmin(dist)

            cv2.waitKey(0)
            radius_med = np.median(radius)
            print "Center location: ", centers[ind]
            print "Median radius: ", radius_med
            val_loc = [(centers[ind][0]),(centers[ind][1])]
            """
            #k_y = 0.22*308/(self.xA*1000)
            #k_z = 0.27*308/(self.xA*1000)
            k_y = 0.5*308/(self.xA*1000)
            k_z = 0.5*308/(self.xA*1000)
            stem_y = int(centers[ind][0]+k_y*(centers[ind][0]-1920/2))
            stem_z = int(centers[ind][1]+k_z*(centers[ind][1]-1080/2))
            img_circ_med = cv_image.copy()
            for n in range(0,len(B_circles[0][:][0])):
                cv2.circle(img_circ_med,(B_circles[0][n][0],B_circles[0][n][1]), B_circles[0][n][2],(0,255,0), 2, cv2.CV_AA)
            cv2.circle(img_circ_med,(centers[ind][0],centers[ind][1]), radius_med,
                    (0,255,0), 2, cv2.CV_AA)
            cv2.circle(img_circ_med,(stem_y,stem_z), 5,
                    (255,0,0), 2, cv2.CV_AA)
            if self.preview_flag or self.preview_result:
                cv2.imshow('img',img_circ_med)
                cv2.waitKey(0)
            self.image_circ_pub.publish(self.bridge.cv2_to_imgmsg(img_circ_med, "bgr8"))
            self.image_output.publish(self.bridge.cv2_to_imgmsg(img_circ_med, "bgr8"))
            if self.save_result:
                cv2.imwrite('%svalveid_result.png' % (self.indir),img_circ_med)
            #cv2.imwrite('/home/administrator/idvalve_vis_%s.png' % str(img_count),img_circ_med)
            #cv2.imshow('Image median circle', img_circ_med)
            #cv2.waitKey(0)
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
            cv2.imwrite('/home/administrator/valveID_0_circs.png',cimg)
            rospy.logdebug("Valve in pixels: %s", " ".join(str(x) for x in val_loc))
            """

            # Find camera dimensions wrt the base coordinate system
            camera_y_mx = self.xA*np.arctan(self.camera_fov_h/2)
            camera_y_mn = -1*self.xA*np.arctan(self.camera_fov_h/2)
            camera_z_mx = self.xA*np.arctan(self.camera_fov_v/2)
            camera_z_mn = -1*self.xA*np.arctan(self.camera_fov_v/2)
            rospy.logdebug("Camera ymn/ymx: %s %s", str(camera_y_mn), str(camera_y_mx))
            rospy.logdebug("Camera zmn/zmx: %s %s", str(camera_z_mn), str(camera_z_mx))

            # Convert the valve pixel location
            valve_y = (1-val_loc[0]/1920)*(camera_y_mx-camera_y_mn)+camera_y_mn
            valve_z = (1-val_loc[1]/1080)*(camera_z_mx-camera_z_mn)+camera_z_mn

            self.valve_id = np.array([self.xA, valve_y, valve_z],dtype=np.float32)
            rospy.logdebug("Valve in m: %s", " ".join(str(x) for x in self.valve_id))
            rospy.set_param('valve_ID',[float(self.valve_id[0]), float(self.valve_id[1]), float(self.valve_id[2])])
            err = np.power(valve_y*valve_y+valve_z*valve_z,0.5)
            rospy.loginfo("Error is: %f",err)
            if err < 0.0035:
                rospy.set_param('valve', [float(self.xA),
                                          float(0.0),
                                          float(0.0)])
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
            self.ct5 = self.ct5 + 1
            if self.ct5 > 25:
                rospy.set_param('smach_state','valveNotFound')

        rospy.signal_shutdown('Ending node.')

if __name__ == '__main__':
    try:
        move2op()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("idvalve finished.")

