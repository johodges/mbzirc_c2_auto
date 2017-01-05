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

class centerwrench():
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
        rospy.init_node('centerwrench', anonymous=True)
        
        # Enable shutdown in rospy
        rospy.on_shutdown(self.shutdown)

        # Define parameters
        self.lim_type = 0   # Type of imadjust
        self.n_wr = 6       # Number of wrenches
        self.segment_median_value = 3       # Number for median filtering
        self.segment_area_threshold = 30    # Minimum area threshold
        self.segment_kernel_sz = 8          # Kernel size
        self.save_flag = False              # Should we save images

        # Tweaking parameters
        self.max_circ_diam = 100        # Maximum circle diameter considered
        self.canny_param = [100, 20]    # Canny edge detection thresholds

        # Hardware Parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080

        # Counters
        self.error_counter = 0

        # Establish publishers and subscribers
        self.bridge = CvBridge()
        self.image_output = rospy.Publisher("/output/keyevent_image",Image,
            queue_size=1)
        self.tftree = tf.TransformListener()
        rospy.Subscriber("/bearing", numpy_msg(Floats), self.callback_bearing,
            queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    def shutdown(self):
        """This subroutine runs when the centerwrench node dies. It is not
        really important for this node.
        """
        rospy.sleep(0.1)

    def callback_bearing(self, bearing):
        """This callback occurs whenever a new /bearing topic is published by
        the orient_scan node. This stores the distance to the board.
        """
        self.xA = bearing.data[1]

    def callback(self, data):
        """This callback takes the RGB image and determines the (x,y,z)
        location of the correct wrench.
        """

        def detect_box_edge(img):
            """This subroutine crops the RGB image to remove the gripper and
            the edge of the box if it is visible.
            """
            sz = np.shape(img)
            img = img[0:sz[0]*69/96,0:sz[1]]
            sz = np.shape(img)
            img_edge = cv2.Canny(img,self.canny_param[0],self.canny_param[1])
            nnz = np.zeros([sz[1],1])
            kernel = np.ones((1,5), np.uint8)
            img_edge2 = cv2.dilate(img_edge, kernel, iterations=2)
            for i in range(0,sz[1]):
                tmp = np.count_nonzero(img_edge2[:,i])
                if tmp:
                    nnz[i,0] = tmp
            ind = nnz[:,0].argsort()
            col = ind[sz[1]-1]
            mx = nnz[col,0]
            if mx >= 0.9*sz[0]:
                col = ind[sz[1]-1]
            else:
                col = sz[1]
            img = img[0:sz[0],0:col]
            return img

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
                lims[i,0] = j+20
                if self.lim_type == 0:
                    val = 0; j = 0;
                    while val < one_perc:
                        val = val+hist[254-j]
                        j = j + 1
                    lims[i,1] = 254-j+20
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

        def back_ground_remove(I):
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
            return I3

        def image_segmentation(img1,median_value,area_threshold,kernel_sz):
            """ This subroutine converts the RGB image without background to
            binary. It returns the binary and grayscale images.
            """
            # Convert the image to grayscale
            img2 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            # Apply 2-D median filter to the grayscale image
            img2 = cv2.medianBlur(img2,median_value)
            # Convert the grayscale image to binary using Otsu's method
            ret,final = cv2.threshold(img2, 0, 255,cv2.THRESH_BINARY
                + cv2.THRESH_OTSU)
            # Find contours within the binary image
            (cnts, _) = cv2.findContours(final.copy(), cv2.RETR_LIST,
                cv2.CHAIN_APPROX_SIMPLE)
            # Initialize mask for image
            mask = np.ones(final.shape[:2], dtype="uint8") * 255
            # Loop through each detected contour
            for c in cnts:
                # Ignore contours which are too small to reduce noise
                area = cv2.contourArea(c)
                if area < area_threshold:
                    # Add contour to mask for image
                    cv2.drawContours(mask,[c], -1, 0, -1)
            # Flip bits in the binary image from the bask
            final2 = cv2.bitwise_and(final, final, mask=mask)
            # Close gaps in the image using an ellipse kernel
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                (kernel_sz,kernel_sz))
            remove = cv2.morphologyEx(final2, cv2.MORPH_OPEN, kernel)
            # Invert the image
            remove = (255-remove)
            return remove, img2

        def detect_circle(img_gray_hou):
            """This subroutine detects all the circles in the image, segments
            them into NUMBER_OF_WRENCHES bins and returns the (row,col)
            coordinates of the centers and the mean radius of each group.
            """
            # Detect the circles using a hough transform
            circles = cv2.HoughCircles(img_gray_hou, cv2.cv.CV_HOUGH_GRADIENT,
                1,1,np.array([]),self.canny_param[0],self.canny_param[1],0,
                self.max_circ_diam)
            img_hou_all = cv2.cvtColor(img_gray_hou.copy(), cv2.COLOR_GRAY2BGR)
            center_x = circles[0,:,0]
            center_y = circles[0,:,1]
            radius = circles[0,:,2]
            for n in range(len(circles[0,:,1])):
                cv2.circle(img_hou_all,(center_x[n],center_y[n]), radius[n],
                    (0,0,244), 2, cv2.CV_AA)
            # Establish matrix of features to use for quanitzation
            z = np.transpose(np.vstack((circles[0,:,0],circles[0,:,1])))
            # Run K-means to det. centers and to which group each point belongs
            term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
            flag = cv2.KMEANS_RANDOM_CENTERS
            ret = []
            ret_old = 99999999999
            ret_flag = 0
            for i in range(1,self.n_wr+2):
                ret2, labels, centers = cv2.kmeans(z, i, term_crit, 100, flag)
                print ret2/ret_old
                if ret2/ret_old < 0.1 and i > 2:
                    ret_flag = 1
                if ret_flag == 0:
                    ret.append(ret2)
                    ret_old = ret2
            ret = np.asarray(ret)
            ret_ind = np.argmin(ret)
            k = ret_ind+1
            print "Best number of clusters is: ", k
            print "Best ret is: ", ret[ret_ind]
            ret, labels, centers = cv2.kmeans(z, k, term_crit, 100, flag)
            return centers, img_hou_all, k

        # Convert ROS image to opencv image
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            sz_full = np.shape(img)
            # Crop image to remove gripper and edge of box
            img_crop = detect_box_edge(img.copy())
            # Determine ideal limits for brightness/contrast adjustment
            lims = stretchlim(img_crop)
            # Adjust the brightness/contrast of the RGB image based on limits
            img_adj = imadjust(img_crop.copy(),lims)
            # Remove Background from adjusted brightness/contrast image
            img_remove = back_ground_remove(img_adj.copy())
            # Convert the image to binary
            img_seg, img_gray = image_segmentation(img_remove.copy(),
                self.segment_median_value,self.segment_area_threshold,
                self.segment_kernel_sz)
            sz = np.shape(img)
            # Crop image for circle detection
            img_gray_hou = np.copy(img_gray) 
            # Detect circles
            centers, img_all_circles, k = detect_circle(img_gray_hou)
            # Publish key event image showing all the circles
            self.image_output.publish(self.bridge.cv2_to_imgmsg(
                img_all_circles, "bgr8"))
            rospy.sleep(0.1)
            # Find which cluster is closest to the center
            sz_circs = np.shape(centers)
            centers = centers[centers[:,0].argsort()]
            rospy.logdebug("Center locations:")
            rospy.logdebug(centers)

            cents = centers.copy()
            cents[:,0] = centers[:,0] - sz_full[1]/2
            cents[:,1] = centers[:,1] - sz_full[0]/2
            cents = np.multiply(cents,cents)
            dist = np.zeros([k,1])
            for i in range(0,k):
                dist[i] = np.power(cents[i,0]+cents[i,1],0.5)/2

            dist_loc = np.argmin(dist)
            dist_min = np.array(dist[dist_loc],dtype=np.float32)

            rospy.logdebug("The minimum distance is: %f", dist_min)
            rospy.logdebug("The index of minimum distance is: %f", dist_loc)
            wrench_ind = centers[dist_loc,:]
            rospy.logdebug("Circle closest to center is (row,col): (%f,%f)",
                wrench_ind[0], wrench_ind[1])

            if self.tftree.frameExists(
                "/base_laser") and self.tftree.frameExists("/camera"):
                t = self.tftree.getLatestCommonTime("/base_laser", "/camera")
                posi, quat = self.tftree.lookupTransform("/base_laser", 
                    "/camera", t)
                rospy.logdebug("TF Position from base_link to camera:")
                rospy.logdebug(posi)
                rospy.logdebug("TF Quaternion from base_link to camera:")
                rospy.logdebug(quat)
            tf_x = posi[0]
            tf_y = posi[1]
            tf_z = posi[2]
            ee_position = rospy.get_param('ee_position')
            xA_tf = np.array(self.xA + 0.461 - ee_position[0], dtype=np.float32)
            rospy.set_param('xA',float(self.xA))
            rospy.logdebug("xA: %f", self.xA)
            row = int(round(wrench_ind[1]))
            col = int(round(wrench_ind[0]))
            rospy.logdebug("wrench_id_px (row,col): (%f,%f)", row, col)
            self.wrench_id_px = np.array([row,col],dtype=np.float32)
            camera_y_mx = xA_tf*np.arctan(self.camera_fov_h/2)
            camera_y_mn = -1*xA_tf*np.arctan(self.camera_fov_h/2)
            camera_z_mx = xA_tf*np.arctan(self.camera_fov_v/2)
            camera_z_mn = -1*xA_tf*np.arctan(self.camera_fov_v/2)
            # Convert the wrench pixel location to m
            wrenc_y = (1-(float(col)/self.camera_pix_h))*(camera_y_mx-camera_y_mn)+camera_y_mn
            wrenc_z = (1-(float(row)/self.camera_pix_v))*(camera_z_mx-camera_z_mn)+camera_z_mn
            self.wrench_id_m = np.array([xA_tf, wrenc_y, wrenc_z],dtype=np.float32)
            rospy.logdebug("wrench_id_m (x,y,z): (%f,%f,%f)",
                self.wrench_id_m[0],self.wrench_id_m[1],self.wrench_id_m[2])
            rospy.set_param('wrench_ID',[float(self.wrench_id_px[0]), 
                float(self.wrench_id_px[1])])
            rospy.set_param('wrench_ID_m',[float(self.wrench_id_m[0]), 
                float(self.wrench_id_m[1]), float(self.wrench_id_m[2])])
            rospy.set_param('wrench_ID_dist',[float(dist_min)])
            rospy.loginfo("Wrench pos. in camera coord. (x,y,z): (%f,%f,%f)", 
                self.wrench_id_m[0],self.wrench_id_m[1],self.wrench_id_m[2])
            # Save images if desired
            if self.save_flag:
                cv2.imwrite('~/wrenchID_1_crop.png',img_crop)
                cv2.imwrite('~/wrenchID_2_adj.png',img_adj)
                cv2.imwrite('~/wrenchID_3_bg.png',img_remove)
                cv2.imwrite('~/wrenchID_4_gray.png',img_gray)
                cv2.imwrite('~/wrenchID_5_seg.png',img_seg)
                cv2.imwrite('~/wrenchID_6_allcircles.png',img_all_circles)
            print "4"
            rospy.signal_shutdown('Ending node.')
        except:
            # Increment error counter if bad things happen
            self.error_counter = self.error_counter+1
            if self.error_counter > 100:
                rospy.set_param('smach_state','wrenchNotFound')
                rospy.signal_shutdown('Ending node.')
if __name__ == '__main__':
    try:
        centerwrench()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("idwrench finished.")

