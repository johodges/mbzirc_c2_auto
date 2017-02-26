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
import scipy.cluster.hierarchy as hcluster
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
        self.save_flag = True              # Should we save images
        self.preview_flag = True          # Should we preview images
        self.preview_result = False
        self.all_circles = False
        self.save_result = True
        rospack = rospkg.RosPack()  # Find rospackge locations
        self.indir = str(rospack.get_path('mbzirc_c2_auto')+'/params/')
        self.area_min_thresh = 3000
        self.xA = 0.68
        self.lim_adjust = 50

        # Tweaking parameters
        self.min_circ_diam = 25
        self.max_circ_diam = 150        # Maximum circle diameter considered
        self.canny_param = [100, 40]    # Canny edge detection thresholds

        # Hardware Parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080

        try:
            self.image_count = rospy.get_param('image_count')
        except:
            self.image_count = 0

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
        if bearing.data[1] != 0:
            self.xA = bearing.data[1]

    def callback(self, data):
        """This callback takes the RGB image and determines the (x,y,z)
        location of the correct wrench.
        """

        def detect_box_edge(img):
            offset = 0
            sz = np.shape(img)
            img = img[0:sz[0],offset:sz[1]]
            img[:,0:700,:] = 0
            img[:,1220:,:] = 0
            img[0:150,:,:] = 0
            img[930:1080,:,:] = 0
            """
            sz = np.shape(img)
            img_edge = cv2.Canny(img,self.canny_param[0],self.canny_param[1])
            nnz = np.zeros([sz[1],1])
            kernel = np.ones((1,5), np.uint8)
            img_edge2 = cv2.dilate(img_edge, kernel, iterations=20)
            for i in range(0,sz[1]):
                tmp = np.count_nonzero(img_edge2[:,i])
                if tmp:
                    nnz[i,0] = tmp
            nnz2 = nnz[::-1]
            col2 = np.argmax(nnz[:,0]>700)
            col1 = 1920-offset-np.argmax(nnz2[:,0]>700)

            if col1-col2 > 100:
                #img = img[0:sz[0],col2:col1]
                img = img[0:sz[0],0:col1]
            else:
                img = img[0:sz[0],0:col1]
            """
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

        def detect_circle(img_bg_rm,img_orig):
            """This subroutine detects all the circles in the image, segments
            them into NUMBER_OF_WRENCHES bins and returns the (row,col)
            coordinates of the centers and the mean radius of each group.
            """

            img_train = cv2.imread(self.indir+'wrench_train_image.jpg',0)
            img_detect = img_bg_rm.copy()

            img_gray = cv2.cvtColor(img_detect.copy(), cv2.COLOR_BGR2GRAY)
            (_,img_bw) = cv2.threshold(img_gray,0,255,cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            (cnts, _) = cv2.findContours(img_bw.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            # Initialize mask for image
            mask = np.ones(img_gray.shape[:2], dtype="uint8") * 255
            area_threshold = 10000
            # Loop through each detected contour
            max_area = 0
            current_x = 0
            current_y = 0
            current_dist = 9000
            for c in cnts:
                # Ignore contours which are too small to reduce noise
                area = cv2.contourArea(c)
                if area > area_threshold:
                    # Add contour to mask for image
                    #cv2.drawContours(mask,[c], -1, 0, -1)
                    #cv2.drawContours(img_detect, c, -1, (0,0,255), 3)
                    #print area
                    M = cv2.moments(c)
                    cen2_y = int(M["m01"] / M["m00"])
                    cen2_x = int(M["m10"] / M["m00"])
                    #dist = np.power(np.power(1920/2-cen2_x,2)+np.power(1080/2-cen2_y,2),0.5)
                    dist = np.power(np.power(1920/2-cen2_x,2),0.5)
                    print area, dist, cen2_x
                    if dist < current_dist:
                        (col,row,wid2,len2) = cv2.boundingRect(c)
                        current_dist = dist
            print row, col, len2, wid2
            box1 = np.zeros((1,3))
            box1[0,0] = col
            box1[0,2] = row
            dist_loc = 0
            center_image = img_orig[row:row+len2,col:col+wid2].copy()
            dist_thresh = 10
            img_hou_km = img_orig.copy()
            ct = 0
            # Flip bits in the binary image from the bask
            #cv2.imshow('img_detect2',img_detect[row:row+len2,col:col+wid2])
            #cv2.waitKey(0)

            """






            orb = cv2.ORB()
            
            # USE ORB ON ITS OWN
            #kp1, des1 = orb.detectAndCompute(img_train,None)
            #kp2, des2 = orb.detectAndCompute(img_detect,None)
            # USE FAST + ORB
            
            fast1 = cv2.FastFeatureDetector(1)
            fast2 = cv2.FastFeatureDetector(1)
            kp1 = fast1.detect(img_train,None)
            kp2 = fast2.detect(img_detect,None)
            kp1, des1 = orb.compute(img_train,kp1)
            kp2, des2 = orb.compute(img_detect,kp2)
            
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1,des2)
            matches = sorted(matches, key= lambda x:x.distance)
            print "TOTAL NUMBER OF FEATURES: ", len(kp2)
            print "TOTAL NUMBER OF MATCHES: ", len(matches)
            center_x = []
            center_y = []
            kp = []
            for i in range(0,len(matches)/2):#len(matches)/8):
                idx = matches[i].trainIdx
                center_x.append(kp2[idx].pt[0])
                center_y.append(kp2[idx].pt[1])
                kp.append(kp2[idx])
            if self.preview_flag:
                A3 = cv2.drawKeypoints(img_train,kp1,color=(0,255,0), flags=0)
                cv2.imshow('img',A3)
                cv2.waitKey(0)

            img_hou_km = cv2.drawKeypoints(img_detect.copy(),kp,color=(0,0,255), flags=0)
            if self.preview_flag:
                cv2.imshow('img',img_hou_km)
                print "A2"
                cv2.waitKey(0)
                #cv2.destroyAllWindows()

            #img_hou_all = cv2.cvtColor(img_gray_hou.copy(), cv2.COLOR_GRAY2BGR)
            #img_hou_km = img_orig.copy()#img_hou_all.copy()
            z = np.transpose(np.vstack((np.round(center_x),np.round(center_y))))
            z = np.float32(z)
            term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
            flag = cv2.KMEANS_RANDOM_CENTERS
            ret = []
            ret_old = 99999999999
            ret_flag = 0
            """
            """
            for i in range(1,10):
                print "i = ", i
                ret2, labels, centers = cv2.kmeans(z, i, term_crit, 1000, flag)
                print "ret2: ", ret2
                #if ret2 < 2000000:
                #if ret2 < 100000:
                print "ret2/ret_old", ret2/ret_old
                if ret2 < 100000:# and ret2/ret_old < 0.7:
                    ret_flag = 1
                    break
                if ret_flag == 0:
                    ret.append(ret2)
                    ret_old = ret2
            k = i
            print "Best number of clusters is: ", k
            print "Best ret is: ", ret2
            """
            """
            k = 10
            ret, labels, centers = cv2.kmeans(z, k, term_crit, 1000, flag)
            """
            """
            clusters = hcluster.fclusterdata(z, 20, criterion="distance")
            print np.shape(z)
            print "CLUSTERS: ", clusters
            print np.shape(clusters)
            print np.max(clusters)
            print "ALL CENTERS: ", centers
            [val,cl] = np.histogram(clusters,bins=np.max(clusters)-1)
            print val
            centers2 = np.empty([np.max(clusters),2], dtype=int)
            ct = 0
            for n in range(0,np.max(clusters)-1):
                if val[n] > 1:
                    centers2[ct,:] = [np.mean(z[clusters == n,0]),np.mean(z[clusters == n,1])]
                    ct = ct+1
            centers = centers2[:ct,:]
            for n in range(0,ct):
                print "Centers[n,0], [n,1]: ", centers[n,0], centers[n,1]
                cv2.circle(img_hou_km,(centers[n,0], centers[n,1]), 10, (int(255-n/np.max(clusters)),int(n/np.max(clusters)*255),0), 2, cv2.CV_AA)
            print "CENTERS WITH HEIRARCHIAL: ", centers
            
            cv2.imshow('img',img_hou_km)
            cv2.waitKey(0)

            rospy.sleep(100)
            """
            """
            centers2 = np.empty([len(centers[:,0]),2], dtype=int)
            ct = 0
            for n in range(0,k):
                idx = np.where(labels == n)
                print np.count_nonzero(idx)
                if np.count_nonzero(idx) > 4:
                    centers2[ct,:] = centers[n,:]
                    ct = ct+1
            centers = centers2[:ct,:]
            print "CENTERS AFTER SMALL CLUSTER REMOVAL: ", centers
            k = ct

            centers2 = centers.copy()
            ct = 0
            dist_thresh = (self.camera_pix_h/2)/(self.xA*np.tan(self.camera_fov_h/2))*(0.1)

            print "dist_thresh: ", dist_thresh
            for n in range(0,k):
                cent_dist = centers2.copy()
                cent_dist = cent_dist-cent_dist[n,:]
                cent_dist = np.multiply(cent_dist,cent_dist)
                dist = np.zeros([k,1])
                for i in range(0,k):
                    dist[i] = np.power(cent_dist[i,0]+cent_dist[i,1],0.5)/2
                print "dist: ", dist[:,0]
                dist_loc = np.where(dist[:,0] < dist_thresh)
                print "dist_loc: ", dist_loc
                print "len(dist_loc[0]): ", len(dist_loc[0])
                print "centers[n,:]: ", centers2[n,:]
                ct = 1
                for i in range(0,len(dist_loc[0])):
                    print dist_loc[0][i]
                    if dist_loc[0][i] > n:
                        labels[labels == dist_loc[0][i]] = n
                        ct = ct + 1
                        print "centers[dist_loc[0][i],:]", centers2[dist_loc[0][i],:]
                        centers2[n,:] = centers2[n,:]+centers2[dist_loc[0][i],:]
                        centers2[dist_loc[0][i],:] = [0,0]
                centers2[n,:] = centers2[n,:]/ct
                print "INTERMEDIATE CENTERS: ", centers2
            centers3 = centers.copy()
            
            ct = 0
            for n in range(0,k):
                if centers2[n,0] != 0:
                    centers3[ct,:] = centers2[n,:]
                    labels[labels == n] = ct
                    ct = ct+1
            k = ct
                #dist_loc = np.argmin(dist)
                #dist_min = np.array(dist[dist_loc],dtype=np.float32)
            centers = centers3[:ct,:]
            print "CENTERS AFTER RE-GROUPING BASED ON DISTANCE: ", centers

            box2 = np.empty([len(centers[:,0]),4], dtype=int)
            ct = 0
            for n in range(0,k):
                idx = np.where(labels == n)
                center_x_k = z[idx,0]
                center_y_k = z[idx,1]
                center_x_k = center_x_k[0]
                center_y_k = center_y_k[0]
                colo = np.float(n)/np.float(k)*255
                x_mn = np.min(center_x_k)
                x_mx = np.max(center_x_k)
                y_mn = np.min(center_y_k)
                y_mx = np.max(center_y_k)
                cv2.rectangle(img_hou_km,(x_mn,y_mn),(x_mx,y_mx), (255-colo,colo,0),2,0,0)
                box2[ct,:] = [x_mn,x_mx,y_mn,y_mx]
                ct = ct+1
                for j in range(0,len(center_x_k)):
                    cx = center_x_k[j]
                    cy = center_y_k[j]
                    cv2.circle(img_hou_km,(cx, cy), 5, (255-colo,colo,0), 2, cv2.CV_AA)
            
            box1 = box2[:ct,:]

            #for n in range(0,len(centers)):
                #cv2.circle(img_hou_km,(centers[n][0],centers[n][1]), 20,
                #    (0,0,255), 2, cv2.CV_AA)
            if self.preview_flag:
                cv2.imshow('img',img_hou_km)
                #cv2.waitKey(0)

            # Find which cluster is closest to the center
            sz_circs = np.shape(centers)
            #centers = centers[centers[:,0].argsort()]
            rospy.logdebug("Center locations:")
            rospy.logdebug(centers)

            cents = centers.copy()
            cents[:,0] = centers[:,0] - self.sz_full[1]/2
            cents[:,1] = centers[:,1] - self.sz_full[0]/2

            cents = np.multiply(cents,cents)
            dist = np.zeros([ct,1])

            for i in range(0,ct):
                dist[i] = np.power(cents[i,0]+cents[i,1],0.5)/2

            dist_loc = np.argmin(dist)
            dist_min = np.array(dist[dist_loc],dtype=np.float32)

            rospy.logdebug("The minimum distance is: %f", dist_min)
            rospy.logdebug("The index of minimum distance is: %f", dist_loc)
            wrench_ind = centers[dist_loc,:]
            print "dist_loc: ",dist_loc
            rospy.logdebug("Circle closest to center is (row,col): (%f,%f)",
                wrench_ind[0], wrench_ind[1])

            print "A2"
            print box1[dist_loc,:]
            print "x_mx-x_mn, dist_thresh: ", box1[dist_loc,1]-box1[dist_loc,0], dist_thresh
            #if (box1[dist_loc,1]-box1[dist_loc,0]) > dist_thresh*10:
            #    print "Error, wrench box too big"
            #    return
            center_image = img_orig[box1[dist_loc,2]:box1[dist_loc,3],box1[dist_loc,0]:box1[dist_loc,1]].copy()
            """
            scale_factor = 2
            center_image = cv2.resize(center_image, (0,0), fx=scale_factor, fy=scale_factor);

            center_image_invert = 255-center_image.copy()
            if self.preview_flag:
                cv2.imshow('img',center_image_invert)
                cv2.waitKey(0)
            sz_2 = np.shape(center_image_invert)

            if len(sz_2) != 3:
                center_image_invert = cv2.cvtColor(center_image_invert.copy(), cv2.COLOR_GRAY2BGR)
            # Determine ideal limits for brightness/contrast adjustment
            lims = stretchlim(center_image_invert)
            # Adjust the brightness/contrast of the RGB image based on limits
            img_adj = imadjust(center_image_invert.copy(),lims)
            if self.preview_flag:
                cv2.imshow('img',img_adj)
                print "img_adj"
                cv2.waitKey(0)
            # Remove Background from adjusted brightness/contrast image
            img_remove = back_ground_remove(img_adj.copy(),center_image.copy())
            if self.preview_flag:
                cv2.imshow('img',img_remove)
                print "img_remove"
                cv2.waitKey(0)
            edges = cv2.Canny(img_remove,10,60)
            if self.preview_flag:
                cv2.imshow('img',edges)
                cv2.waitKey(0)
                #cv2.destroyAllWindows()
            minLineLength = 100
            maxLineGap = 50
            lines = cv2.HoughLines(edges,1,np.pi/180,50)
            lines_horz = cv2.HoughLines(edges,1,np.pi/180,20)
            print np.max(lines[:,1])
            print np.max(lines[:,0])
            sz = np.shape(edges)
            horz_line = 0
            vert_line1 = 0
            vert_line2 = sz[1]

            for rho,theta in lines[0]:
                if abs(theta) > 2.8 and abs(theta) < 3.00:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    cv2.line(img_remove,(x1,y1),(x2,y2),(0,0,255),2)

            for rho,theta in lines_horz[0]:
                if abs(theta) > 1.52 and abs(theta) < 1.60:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    cv2.line(img_remove,(x1,y1),(x2,y2),(0,0,255),2)
                
            if self.preview_flag:
                cv2.imshow('img',img_remove)
                cv2.waitKey(0)
                #cv2.destroyAllWindows()

            for rho,theta in lines[0]:
                if abs(theta) > 2.8 and abs(theta) < 3.00:
                    a = np.cos(theta)
                    x0 = a*rho
                    if x0 > vert_line1:
                        vert_theta1 = theta
                        vert_rho1 = rho
                        vert_line1 = x0
                    if x0 < vert_line2:
                        vert_theta2 = theta
                        vert_rho2 = rho
                        vert_line2 = x0
            for rho,theta in lines_horz[0]:
                if abs(theta) > 1.52 and abs(theta) < 1.60:
                    b = np.sin(theta)
                    y0 = b*rho
                    if y0 > horz_line and y0 < sz[0]-50:
                        horz_theta = theta
                        horz_rho = rho
                        horz_line = y0
            #HORIZONTAL LINE
            a = np.cos(horz_theta)
            b = np.sin(horz_theta)
            x0 = a*horz_rho
            y0 = b*horz_rho
            x1 = float(x0 + 1000*(-b))
            y1 = float(y0 + 1000*(a))
            x2 = float(x0 - 1000*(-b))
            y2 = float(y0 - 1000*(a))
            horz_m = (y2-y1)/(x2-x1)
            horz_b = y1-horz_m*x1

            #RIGHT VERTICAL LINE
            a = np.cos(vert_theta1)
            b = np.sin(vert_theta1)
            x0 = a*vert_rho1
            y0 = b*vert_rho1
            x1 = float(x0 + 1000*(-b))
            y1 = float(y0 + 1000*(a))
            x2 = float(x0 - 1000*(-b))
            y2 = float(y0 - 1000*(a))
            vert_x1 = x1
            vert_y1 = y1
            vert_m1 = (y2-y1)/(x2-x1)
            vert_b1 = y1-vert_m1*x1
            ybot1 = sz[0]
            xbot1 = (ybot1-vert_b1)/vert_m1
            x_int1 = (vert_b1 - horz_b)/(horz_m-vert_m1)
            y_int1 = vert_m1 * x_int1 + vert_b1

            #LEFT VERTICAL LINE
            a = np.cos(vert_theta2)
            b = np.sin(vert_theta2)
            x0 = a*vert_rho2
            y0 = b*vert_rho2
            x1 = float(x0 + 1000*(-b))
            y1 = float(y0 + 1000*(a))
            x2 = float(x0 - 1000*(-b))
            y2 = float(y0 - 1000*(a))
            vert_x2 = x1
            vert_y2 = y1
            vert_m2 = (y2-y1)/(x2-x1)
            vert_b2 = y1-vert_m2*x1
            x_int2 = (horz_b - vert_b2)/(horz_m-vert_m2)
            y_int2 = vert_m2 * x_int2 + vert_b2
            ybot2 = sz[0]
            xbot2 = (ybot2-vert_b2)/vert_m2
            x_int2 = (vert_b2 - horz_b)/(horz_m-vert_m2)
            y_int2 = vert_m2 * x_int2 + vert_b2
            d = np.power(np.power(x_int1-x_int2,2)+np.power(y_int1-y_int2,2),0.5)
            d_tip1 = np.power(np.power(x_int1-xbot1,2)+np.power(y_int1-ybot1,2),0.5)
            d_tip2 = np.power(np.power(x_int2-xbot2,2)+np.power(y_int2-ybot2,2),0.5)
            x_tip1 = x_int1-(x_int1-xbot1)*d/d_tip1
            x_tip2 = x_int2-(x_int2-xbot2)*d/d_tip2
            y_tip1 = y_int1-(y_int1-ybot1)*d/d_tip1
            y_tip2 = y_int2-(y_int2-ybot2)*d/d_tip2

            #CALCULATE CENTER
            cent_x = (x_int1+x_int2+x_tip1+x_tip2)/4
            cent_y = (y_int1+y_int2+y_tip1+y_tip2)/4
            cent_y = ybot2 - 200

            #DRAW LINES
            """
            cv2.line(center_image,(int(x_int1),int(y_int1)),(int(x_int2),int(y_int2)),(0,0,255),2)
            cv2.line(center_image,(int(xbot1),int(ybot1)),(int(x_int1),int(y_int1)),(0,0,255),2)
            cv2.line(center_image,(int(xbot2),int(ybot2)),(int(x_int2),int(y_int2)),(0,0,255),2)
            cv2.circle(center_image,(int(cent_x),int(cent_y)),5,(0,0,255),-1,cv2.CV_AA)
            """

            #SCALE BACK TO FULL SIZE IMAGE AND COORDINATES
            cent_x = cent_x/scale_factor+box1[dist_loc,0]
            cent_y = cent_y/scale_factor+box1[dist_loc,2]
            x_int1 = x_int1/scale_factor+box1[dist_loc,0]
            y_int1 = y_int1/scale_factor+box1[dist_loc,2]
            x_int2 = x_int2/scale_factor+box1[dist_loc,0]
            y_int2 = y_int2/scale_factor+box1[dist_loc,2]
            xbot1 = xbot1/scale_factor+box1[dist_loc,0]
            ybot1 = ybot1/scale_factor+box1[dist_loc,2]
            xbot2 = xbot2/scale_factor+box1[dist_loc,0]
            ybot2 = ybot2/scale_factor+box1[dist_loc,2]
            x_tip1 = x_tip1/scale_factor+box1[dist_loc,0]
            y_tip1 = y_tip1/scale_factor+box1[dist_loc,2]
            x_tip2 = x_tip2/scale_factor+box1[dist_loc,0]
            y_tip2 = y_tip2/scale_factor+box1[dist_loc,2]

            if (abs(xbot1-xbot2)) < dist_thresh/2:
                return
            print x_int1, y_int1, x_int2, y_int2
            print np.shape(img_hou_km)
            cv2.line(img_hou_km,(int(x_int1),int(y_int1)),(int(x_int2),int(y_int2)),(0,255,0),2)
            cv2.line(img_hou_km,(int(x_tip1),int(y_tip1)),(int(x_int1),int(y_int1)),(0,255,0),2)
            cv2.line(img_hou_km,(int(x_tip2),int(y_tip2)),(int(x_int2),int(y_int2)),(0,255,0),2)
            cv2.circle(img_hou_km,(int(cent_x),int(cent_y)),5,(0,255,0),-1,cv2.CV_AA)
            cv2.imwrite('/home/ugv/center_wrench.png',center_image)
            #img_hou_km = img_orig.copy()
            return [cent_x,cent_y], img_hou_km, ct

        def quantize_image(img):
            """This subroutine quantizes an image into 3 kmeans color groups
            """
            sz = np.shape(img)
            z = np.float32(img.copy())
            z = z.reshape((sz[0]*sz[1],1))
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            ret,labels,centers = cv2.kmeans(z,2,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
            mean_sort = np.argsort(centers.flatten())
            z = z.reshape((sz[0],sz[1]))
            labels = labels.reshape((sz[0],sz[1]))
            A = img.copy()
            B = img.copy()
            A[labels != mean_sort[0]] = 0
            A[labels == mean_sort[0]] = 255#centers[mean_sort[0]]
            B[labels != mean_sort[1]] = 0
            B[labels == mean_sort[1]] = 255#centers[mean_sort[1]]
            kernel1 = np.ones((3,3), np.uint8)
            img_edge0 = cv2.erode(A, kernel1, iterations=1)
            img_edge1 = cv2.dilate(img_edge0, kernel1, iterations=30)
            img_edge2 = cv2.erode(img_edge1, kernel1, iterations=20)
            im_floodfill = img_edge2.copy()
            h, w = img_edge2.shape[:2]
            mask = np.zeros((h+2, w+2), np.uint8)
            tmp1 = 0; tmp2 = 0;
            while im_floodfill[tmp1,tmp2] == 255:
                tmp1 = tmp1+10
                tmp2 = tmp2+10
            cv2.floodFill(im_floodfill, mask, (tmp1,tmp2), 255);
 
            # Invert floodfilled image
            im_floodfill_inv = cv2.bitwise_not(im_floodfill)
 
            # Combine the two images to get the foreground.
            im_out = img_edge2 | im_floodfill_inv
            A = im_out #img_edge2
            return A, B

        def drawMatches(img1, kp1, img2, kp2, matches):
            """
            My own implementation of cv2.drawMatches as OpenCV 2.4.9
            does not have this function available but it's supported in
            OpenCV 3.0.0

            This function takes in two images with their associated 
            keypoints, as well as a list of DMatch data structure (matches) 
            that contains which keypoints matched in which images.
    
            An image will be produced where a montage is shown with
            the first image followed by the second image beside it.
    
            Keypoints are delineated with circles, while lines are connected
            between matching keypoints.
    
            img1,img2 - Grayscale images
            kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
                    detection algorithms
            matches - A list of matches of corresponding keypoints through any
                      OpenCV keypoint matching algorithm
            """
            
            # Create a new output image that concatenates the two images together
            # (a.k.a) a montage
            rows1 = img1.shape[0]
            cols1 = img1.shape[1]
            rows2 = img2.shape[0]
            cols2 = img2.shape[1]
            
            out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')
            
            # Place the first image to the left
            out[:rows1,:cols1] = np.dstack([img1, img1, img1])
        
            # Place the next image to the right of it
            out[:rows2,cols1:] = np.dstack([img2, img2, img2])
        
            # For each pair of points we have between both images
            # draw circles, then connect a line between them
            for mat in matches:
        
                # Get the matching keypoints for each of the images
                img1_idx = mat.queryIdx
                img2_idx = mat.trainIdx
        
                # x - columns
                # y - rows
                (x1,y1) = kp1[img1_idx].pt
                (x2,y2) = kp2[img2_idx].pt
        
                # Draw a small circle at both co-ordinates
                # radius 4
                # colour blue
                # thickness = 1
                cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
                cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)
        
                # Draw a line in between the two points
                # thickness = 1
                # colour blue
                cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)
        
        
            # Show the image
            cv2.imshow('Matched Features', out)
            cv2.waitKey(0)
            cv2.destroyWindow('Matched Features')
    
            # Also return the image if you'd like a copy
            return out

        # Convert ROS image to opencv image
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if True:
            self.sz_full = np.shape(img)
            if self.save_flag:
                cv2.imwrite('%scresent_%s_%s.png' % (self.indir, str(self.image_count), str(int(1000*self.xA))),img)
            self.image_count = self.image_count+1
            rospy.set_param('image_count',self.image_count)
            # Crop image to remove gripper and edge of box
            img_crop = detect_box_edge(img.copy())
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
            img_gray = cv2.cvtColor(img_remove.copy(), cv2.COLOR_BGR2GRAY)
            img_gray_orig = 255-cv2.cvtColor(img_crop.copy(), cv2.COLOR_BGR2GRAY)
            img_gray[np.where(img_gray != [255])] = img_gray_orig[np.where(img_gray != [255])]
            img_gray_rgb = cv2.cvtColor(255-img_gray.copy(), cv2.COLOR_GRAY2BGR)
            #img_gray[np.where(img_gray == [255])] = img_gray_orig[np.where(img_gray == [255])]
            #img_remove[np.where((img_remove<254).all(axis=2))] = img_crop[np.where((img_remove<254).all(axis=2))]
            #img_remove = 255 - img_remove
            """
            # Convert the image to binary
            sz = np.shape(img)
            img_gray = cv2.cvtColor(img_remove.copy(), cv2.COLOR_BGR2GRAY)
            [A,B] = quantize_image(img_gray.copy())
            img_gray_orig = cv2.cvtColor(img_crop.copy(), cv2.COLOR_BGR2GRAY)
            A[A == 255] = img_gray_orig[A == 255]
            if self.preview_flag:
                cv2.imshow('img',A)
                print "A"
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            # Crop image for circle detection
            img_gray_hou = np.copy(img_gray) 
            # Detect circles
            #centers, img_all_circles, k = detect_circle(img_gray_hou, img_crop.copy())
            """
            wrench_ind, img_all_circles, k = detect_circle(img_gray_rgb.copy(),img.copy())
            if self.preview_flag:
                cv2.imshow('img',img_all_circles)
                cv2.waitKey(0)
            # Publish key event image showing all the circles
            self.image_output.publish(self.bridge.cv2_to_imgmsg(
                img_all_circles, "bgr8"))
            if self.preview_flag or self.preview_result:
                cv2.imshow('img',img_all_circles)
                cv2.waitKey(0)
            if self.save_result:
                cv2.imwrite('%scresent_%s_%s_result.png' % (self.indir, str(self.image_count), str(int(1000*self.xA))),img_all_circles)
            rospy.sleep(0.1)
            ee_position = rospy.get_param('ee_position')
            #sssxA_tf = np.array(self.xA + 0.461 - ee_position[0], dtype=np.float32)
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
            #rospy.set_param('wrench_ID_dist',[float(dist_min)])
            rospy.loginfo("Wrench pos. in camera coord. (x,y,z): (%f,%f,%f)", 
                self.wrench_id_m[0],self.wrench_id_m[1],self.wrench_id_m[2])
            # Save images if desired
            """
            if self.save_flag:
                cv2.imwrite('~/wrenchID_1_crop.png',img_crop)
                cv2.imwrite('~/wrenchID_2_adj.png',img_adj)
                cv2.imwrite('~/wrenchID_3_bg.png',img_remove)
                cv2.imwrite('~/wrenchID_4_gray.png',img_gray)
                cv2.imwrite('~/wrenchID_5_seg.png',img_seg)
                cv2.imwrite('~/wrenchID_6_allcircles.png',img_all_circles)
            print "4"
            """
            rospy.signal_shutdown('Ending node.')
        else:
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

