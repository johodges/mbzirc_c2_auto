#!/usr/bin/env python

""" circle_wrench.py - Version 1.0 2016-11-15

    Based on the R. Patrick Goebel's good_feature.py demo code

    Locate HoughCircles feature closest to center of the image to track in a video stream.

    Created for the MBZIRC competition
    author: LUAN CONG DOAN - luandoan@vt.edu
    CMS Lab

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import cv2
import cv2.cv as cv
from cv2 import BORDER_DEFAULT
from cv2.cv import CV_HOUGH_GRADIENT
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from base_detector import BaseDetector
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, Twist
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import tf
import math

class CircleWrench(BaseDetector):
    def __init__(self, node_name):
	super(CircleWrench, self).__init__(node_name)

	# Should I show text on the display?
        self.show_text = rospy.get_param("~show_text", True)

        # How big should the feature points be (in pixels)?
        self.feature_size = rospy.get_param("~feature_size", 1)
	
    	# Initialize key variables
	self.max_radius = 200	# maximum circle diameter
	self.canny_param = [100, 30]	# Canny Edge detection thresholds
	self.seg_mvalue  = 3
	self.seg_threshold = 30
	self.seg_ksize = 8
        self.feature_point = list()
        self.tracked_point = list()

    def process_image(self, cv_image):
	try:
	    self.img = cv2.imread('/home/luan/catkin_ws/mbzirc_c2_auto/arm_nav_correct/scripts/wrench.png',1)
	    # determine the ideal limits
	    lims = stretchlim(self.img, 0)
	    # adjust the brightness
	    img_adjust = imgadjust(img_adjust,lims)
	    # Remove backgrouimgnd from captured image
	    self.img = self.remove_background(img_adjust)
	    # Convert image to binary image
	    img_seg, img_gray = image_segmentation(self.img, self.seg_mvalue, self.seg_threshold, self.seg_ksize)
	    # define image size
	    #sz = np.shape(img)
	    # narrow the image 
	    img_crop = img_gray  #[self.frame_width/3:2*self.frame_width/3, self.frame_height/3:2*self.frame_height/3]
	    # detect circles
	    feature_point = detect_cirlces(img_crop)
		    
	    if feature_point is not None and len(feature_point) > 0:
           	# Draw the center of the  circle
               	cv2.circle(self.marker_image, (feature_point[0], feature_point[1]), self.feature_size, (0, 0, 255, 0), cv.CV_FILLED, 8, 0)
            	# Draw the outer circle
            	cv2.circle(self.marker_image, (feature_point[0], feature_point[1]), feature_point[2], (0, 255, 0, 0), self.feature_size, 8, 0)

            	# Convert feature_point from image coordinates to world coordinates
            	feature_point = np.dot(np.linalg.pinv(self.projectionMatrix), np.array([feature_point[0],feature_point[1], 1]))
            	feature_point = feature_point / feature_point[2]
            	feature_point = np.array((feature_point[0], feature_point[1]))
            	# Provide self.tracked_point to publish_poi to be published on the /wlocs topic
            	self.tracked_point = feature_point
	except:
	    pass
	return cv_image	

    def stretchlim(img,flag):
	# determine the size of image
	sz = np.shape(img)
	num_of_pxs = sz[0]*sz[1]
	one_perc = math.floor(num_of_pxs*0.01)
	lims = np.zeros((sz[2],2))
	# compute the lower/upper 1% threshold for each channel
	for i in range(0,sz[2]):
	    hist,bins = np.histogram(img[:,:,i].ravel(), 255, [0,255])
	    val = 0	
	    j = 0
	    while val < one_perc:
		val = val + hist[j]
	 	j = j + 1
	    lims[i,0] = j
	    if flag == 0:
		val = 0; j = 0;
		while val < one_perc:
		    val = val + hist[254 - j]
		    j = j + 1
		lims[i,1] = 254 - j + 20
	    if flag == 1:
		lims[i,1] = 255
	return lims

    def imgajust(img,lims):
	img2 = np.copy(img)
	sz = np.shape(img2)
	for i in range(0,sz[2]):
	    I2 = img2[:,:,i]
	    I2[I2 > lims[i,1]] = lims[i,1]
	    I2[I2 < lims[i,0]] = lims[i,0]
	    img2[:,:,i] = (I2-lims[i,0])/(lims[i,1]-lims[i,0])*255
	return img2


    def remove_background(I):
	sz = np.shape(I)
	i = np.zeros((sz[2],1))
	I2 = I.copy()
	for j in range(0,sz[2]):
	    hist, bins = np.histogram(I[:,:,j].ravel(), 255, [0,255])
	    I3 = I[:,:,j].copy()
	    i[j] = np.argmax(hist)
	    I3[I3 > 255 - i[j]*0.5] = 255 - i[j]*0.5
	    I3 = I3 + 0.5*i[j]
	    I2[:,:,j] = I3
	return I2

    def image_seg(img, mvalue, area_threshold, ksize):
	# convert the image to grayscale
	img2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	# apply 2D median filter to the gray scale image
	img2 = cv2.medianBlur(img2, mvalue)
	# convert the grayscale image to binary using Otsu's method
	ret, final = cv2.threshold(img2, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
	# find the contour within the binary image
	(cnts, _) = cv2.findContours(final.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	# initialize mask for image
	mask = np.ones(final.shape[:2], dtype="uint8")*255
	# loop through each detected contour
	for c in cnts:
	    # ignore contours which are too small to reduce noise
	    area = cv2.contourArea(c)
	    if area < area_threshold:
		# add contours to mask for image
		cv2.drawContours(mask, [c], -1,0,0-1)
	# Flip bits in the binary image from the bask
	final2 = cv2.bitwise_and(final, final, mask=mask)
	# Close gaps in the image using an ellipse kernel
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
	remove = cv2.morphologyEx(final2, cv2.MORPH_OPEN, kernel)
	# Inverse the image
	remove = (255 - remove)
	return remove, img2

    def detect_circle(self, img_crop):
	locs = list()
	loc = list()
	# detect circle using hough transform 
	circles = cv2.HoughCircles(img_crop, cv2.cv.CV_HOUGH_GRADIENT,1,1,np.array([]),self.canny_param[0], self.canny_param[1], 0, self.max_radius)
	if circles is not None and len(cirles) > 0:
	    # convert cicles params to Numpy array
	    locs = np.float32(circles).reshape(-1,3)
	    # compute the circles closest to the camera's center
	    #dists = np.sqrt((locs[:,0]-self.frame_width/2)**2 + (locs[:,1]-self.frame_height/2)**2)
	    # sorted distances
	    #sorted_dists_idx = np.argsort(dists)
	    #sorted_dists_point = locs[sorted_dists_idx]
	    #loc = sorted_dists_point[0]
	    loc = locs[0]
	return loc
	
    def convert_image(self, ros_image):
	try:
	    cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
	    return np.array(cv_image, dtype=np.uint8)
	except CvBridgeError, e:
	    print e

    def cleanup(self):
	print "Shutting down vision node."
	cv2.destroyAllWindows()

    def shutdown(self):
	rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        node_name = "circle_wrench"
        CircleWrench(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down the Circle Wrench node."
        cv.DestroyAllWindows()

