#!/usr/bin/env python

"""centerwrench.py - Version 1.0 2016-11-17
Author: Jonathan Hodges

This software chooses the correct wrench for the MBZIRC UGV challenge in an RGB
image and outputs an estimate of its 3D location in space relative to the
camera [x,y,z].

Subscribers:
    /bearing: array containing [angle,x_med,xmn,xmx,ymn,ymx,target_x,target_y]
    /usb_cam/image_raw: raw image from webcam on gripper
    /tf: transform tree - stored using tf.TransformListener()

Publishers:
    /output/keyevent_image: image containing key events in the challenge. This
        code publishes the circles from k-means to the topic.

Parameters:
    wrench_ID: location of wrench in camera px coordinates [row,col]
    wrench_ID_m: location of wrench in meters in camera coordinates [x,y,z]
    wrench_ID_dist: distance to wrench in meters
    ee_position: current position of end effector in base_link coordinates
    xA: distance to objet in LIDAR coordinates
    smach_state: status for state machine

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html
    
"""

import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point
from geometry_msgs.msg import Quaternion, Twist
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
import math
from scipy.cluster import vq
import scipy.stats

class maskwrench():
    """MBZIRC Challenge 2 wrench visual servoing routine.

    This software chooses the correct wrench for the MBZIRC UGV challenge in an
    RGB image and outputs an estimate of its 3D location in space relative to
    the camera [x,y,z].

    Attributes:
        lim_type: Type of imadjust limits to use
        n_wr: Number of wrenches
        segment_median_value: Size of median filter kernel
        segment_area_threshold: Minimum size of blob to be a wrench
        segment_kernel_sz: Radius of disk kernel
        save_flag: Should processed images be saved
        max_circ_diam: Maximum circle diameter considered
        canny_param: Canny edge detection thresholds
        camera_fov_h: Horizontal field of view of camera in radians
        camera_fov_v: Vertical field of view of camera in radians
        camera_pix_h: Horizontal field of view of camera in pixels
        camera_pix_v: Vertical field of view of camera in pixels
        error_counter: Error counter
        bridge: OpenCV bridge
        tftree: TF tree
        image_output: Publisher to key event image topic
        xA: Distance to board
        wrench_id_m: location of wrench in meters in camera coordinates [x,y,z]
        wrench_id_px: location of wrench in camera px coordinates [row,col]

    Subscribers:
        /bearing: array containing 
            [angle,x_med,xmn,xmx,ymn,ymx,target_x,target_y]
        /usb_cam/image_raw: raw image from webcam on gripper
        /tf: transform tree - stored using tf.TransformListener()

    Publishers:
        /output/keyevent_image: image containing key events in the challenge.
            This code publishes the circles from k-means to the topic.

    Parameters:
        wrench_ID: location of wrench in camera px coordinates [row,col]
        wrench_ID_m: location of wrench in meters in camera coordinates [x,y,z]
        wrench_ID_dist: distance to wrench in meters
        ee_position: current position of end effector in base_link coordinates
        xA: distance to objet in LIDAR coordinates
        smach_state: status for state machine

"""

    def __init__(self):
        """This initializes the various attributes in the class.
        
        A few key tasks are achieved in the initializer function:
            1. Define parameters for use in the computer vision algorithms
            2. Establish publishers and subscribers

        """
        # Name this node, it must be unique
        rospy.init_node('mask_wrench', anonymous=True)
        rospack = rospkg.RosPack()  # Find rospackge locations
        self.indir = str(rospack.get_path('mbzirc_c2_auto')+'/params/')
        
        # Enable shutdown in rospy
        rospy.on_shutdown(self.shutdown)

        # Define parameters
        self.lim_type = 0           # Type of imadjust
        self.preview_flag = True   # Should we preview images
        self.preview_result = True
        self.save_result = True
        self.lim_adjust = 60

        # Tweaking parameters
        self.canny_param = [10, 40] # Canny edge detection thresholds

        # Establish publishers and subscribers
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    def shutdown(self):
        """This subroutine runs when the centerwrench node dies. It is not
        really important for this node.
        """
        rospy.sleep(0.1)

    def callback(self, data):
        """This callback takes the RGB image and determines the (x,y,z)
        location of the correct wrench.
        """

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

        def imadjust(img,lims):
            """This subroutine adjusts the intensities in each channel of the
            RGB image using the limits supplied by stretchlim. Returns the
            adjusted image.
            """
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
            I_old[I3 == 255] = 255
            return I_old

        # Convert ROS image to opencv image
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if True:
            self.sz_full = np.shape(img)
            # Crop image to remove gripper and edge of box
            to_crop = [int(self.sz_full[0]/3),int(self.sz_full[1]/3)]
            img_crop = img[0:to_crop[0]*2,to_crop[1]:to_crop[1]*2].copy()
            img_crop_invert = 255-img_crop.copy()
            if self.preview_flag:
                cv2.imshow('img',img_crop_invert)
                cv2.waitKey(0)

            # Determine ideal limits for brightness/contrast adjustment
            lims = stretchlim(img_crop_invert)
            # Adjust the brightness/contrast of the RGB image based on limits
            img_adj = imadjust(img_crop_invert.copy(),lims)
            if self.preview_flag:
                cv2.imshow('img',img_adj)
                print "img_adj"
                cv2.waitKey(0)
            # Remove Background from adjusted brightness/contrast image
            img_remove = back_ground_remove(img_adj.copy(),img_crop.copy())
            if self.preview_flag:
                cv2.imshow('img',img_remove)
                print "img_remove"
                cv2.waitKey(0)
            edges = cv2.Canny(img_remove,10,60)
            if self.preview_flag:
                cv2.imshow('img',edges)
                print "edges"
                cv2.waitKey(0)
            kernel1 = np.ones((3,3), np.uint8)
            img_edge1 = cv2.dilate(edges, kernel1, iterations=10)

            im_floodfill = img_edge1.copy()
            h, w = img_edge1.shape[:2]
            mask = np.zeros((h+2, w+2), np.uint8)
            cv2.floodFill(im_floodfill, mask, (0,0), 255);
 
            # Invert floodfilled image
            im_floodfill_inv = cv2.bitwise_not(im_floodfill)
 
            # Combine the two images to get the foreground.
            im_out = img_edge1 | im_floodfill_inv

            img_edge2 = cv2.erode(im_out, kernel1, iterations=9)

            if self.preview_flag:
                cv2.imshow('img',img_edge2)
                print "im_out"
                cv2.waitKey(0)
            img_mask = cv2.cvtColor(img_edge2.copy(), cv2.COLOR_GRAY2BGR)

            img_mask_edge = cv2.Canny(img_mask,self.canny_param[0],self.canny_param[1])
            lines = cv2.HoughLines(img_mask_edge,1,np.pi/180,25)
            sz = np.shape(img_edge2); vert_line1 = sz[1]/2; vert_line2 = sz[1]/2;
            img_mask_edge = cv2.cvtColor(img_mask_edge.copy(), cv2.COLOR_GRAY2BGR)
            for rho,theta in lines[0]:
                if abs(theta) > 2.80 and abs(theta) < 3.14:
                    a = np.cos(theta); b = np.sin(theta);
                    x0 = a*rho; y0 = b*rho
                    x1 = float(x0 + 1000*(-b)); y1 = float(y0 + 1000*(a));
                    x2 = float(x0 - 1000*(-b)); y2 = float(y0 - 1000*(a));
                    m = (y2-y1)/(x2-x1); b = y1-m*x1;
                    ybot = sz[0]; xbot = (ybot-b)/m
                    if xbot < vert_line1:
                        vert_line1 = xbot; m1 = m; b1 = b; x1_1 = x1; y1_1 = y1;
                    if xbot > vert_line2:
                        vert_line2 = xbot; m2 = m; b2 = b; x2_1 = x1; y2_1 = y1;

            #LEFT VERTICAL LINE
            ybot1 = sz[0]; xbot1 = (ybot1-b1)/m1;
            cv2.line(img_mask_edge,(int(xbot1),int(ybot1)),(int(x1_1),int(y1_1)),(0,0,255),2)

            #RIGHT VERTICAL LINE
            ybot2 = sz[0]; xbot2 = (ybot2-b2)/m2;
            cv2.line(img_mask_edge,(int(xbot2),int(ybot2)),(int(x2_1),int(y2_1)),(0,0,255),2)
            img[:,:,:] = 0
            img[0:to_crop[0]*2,to_crop[1]:to_crop[1]*2] = img_mask.copy()
            if self.preview_flag or self.preview_result:
                cv2.imshow('img',img)
                cv2.waitKey(0)
            print "SLOPES m1, m2, average: ", m1, m2, (m1+m2)/2
            if self.save_result:
                cv2.imwrite('%swrench_mask.png' % (self.indir),img)
            rospy.set_param('wrench_slope',[m1,m2,(m1+m2)/2])
            rospy.set_param('wrench_mask_file',self.indir+'wrench_mask.png')
            rospy.signal_shutdown('Ending node.')

if __name__ == '__main__':
    try:
        maskwrench()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("mask_wrench finished.")

