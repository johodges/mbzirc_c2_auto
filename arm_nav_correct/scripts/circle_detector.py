#!/usr/bin/env python

""" circle_detector.py - Version 1.0 2016-02-23

    Based on the R. Patrick Goebel's good_feature.py demo code

    Locate HoughCircles feature closest to center of the image to track in a video stream.

    Created for MBZIRC wrench grasping

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
import numpy as np
from cv2 import BORDER_DEFAULT
from cv2.cv import CV_HOUGH_GRADIENT
from base_detector import BaseDetector

class CircleDetector(BaseDetector):
    def __init__(self, node_name):
        super(CircleDetector, self).__init__(node_name)

        # Should I show text on the display?
        self.show_text = rospy.get_param("~show_text", True)

        # How big should the feature points be (in pixels)?
        self.feature_size = rospy.get_param("~feature_size", 1)

        # Filter parameters
        self.medianBlur_ksize = rospy.get_param("~medianBlur_ksize", 9)
        self.GaussianBlur_ksize_width = rospy.get_param("~GaussianBlur_ksize_width", 15)
        self.GaussianBlur_ksize_height = rospy.get_param("~GaussianBlur_ksize_height", 15)
        self.GaussianBlur_sigmaX = rospy.get_param("~GaussianBlur_sigmaX", 0)
        self.GaussianBlur_sigmaY = rospy.get_param("~GaussianBlur_sigmaY", 0)

        # Store all the feature parameters together for passing to filters
        self.medianBlur_params = dict(ksize = self.medianBlur_ksize)
        self.GaussianBlur_params = dict(ksize = (self.GaussianBlur_ksize_width,self.GaussianBlur_ksize_height),
                                        sigmaX = self.GaussianBlur_sigmaX,
                                        sigmaY = self.GaussianBlur_sigmaY,
                                        borderType = BORDER_DEFAULT)
        # HoughCircles features parameters
        self.HoughCircles_dp = rospy.get_param("~HoughCircles_dp", 1)
        self.HoughCircles_minDist = rospy.get_param("~HoughCircles_minDist", 36)
        self.HoughCircles_param1 = rospy.get_param("~HoughCircles_param1", 200)
        self.HoughCircles_param2 = rospy.get_param("~HoughCircles_param2", 25)
        self.HoughCircles_minRadius = rospy.get_param("~HoughCircles_minRadius", 20)
        self.HoughCircles_maxRadius = rospy.get_param("~HoughCircles_maxRadius", 400)

        # Store all parameters together for passing to the HoughCircle detector
        self.HoughCircles_params = dict(method = CV_HOUGH_GRADIENT,
                                        dp = self.HoughCircles_dp,
                                        minDist = self.HoughCircles_minDist,
                                        param1 = self.HoughCircles_param1,
                                        param2 = self.HoughCircles_param2,
                                        minRadius =self.HoughCircles_minRadius,
                                        maxRadius = self.HoughCircles_maxRadius)

        # Initialize key variables
        self.feature_point = list()
        self.tracked_point = list()
	self.radius = 0
        self.mask = None

	cv_image = cv2.imread('/home/luan/catkin_ws/mbzirc_c2_auto/arm_nav_correct/scripts/wrench.png',1)

    def process_image(self, cv_image):
        try:
            # Mask image
            self.mask = np.zeros_like(cv_image)
            x, y, w, h = self.frame_width / 10, self.frame_height / 10, self.frame_width / 2, self.frame_height / 2
            self.mask[h-y:y+h, w-x:x+w] = 255
            masked_image = cv2.bitwise_and(cv_image, self.mask)

            # Create a grey scale version of the image
            grey = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

            # Equalize the histogram to reduce lighting effects
            grey = cv2.equalizeHist(grey)
	    cv2.imshow(grey)

            # Remove salt-and-pepper, and white noise by using a combination of a median and Gaussian filter
            grey = cv2.medianBlur(grey, **self.medianBlur_params)
            grey = cv2.GaussianBlur(grey, **self.GaussianBlur_params)

            # Get the HoughCircles feature closest to the image's center
            feature_point = self.get_feature_point(grey)

            if feature_point is not None and len(feature_point) > 0:
                # Draw the center of the circle
                cv2.circle(self.marker_image, (feature_point[0], feature_point[1]), self.feature_size, (0, 0, 255, 0), cv.CV_FILLED, 8, 0)
                # Draw the outer circle
                cv2.circle(self.marker_image, (feature_point[0], feature_point[1]), feature_point[2], (0, 255, 0, 0), self.feature_size, 8, 0)
		# record the circle radius
		self.radius = (feature_point[2]*17)/60
		
		# Show the location of circle on image frame
		font_face = cv2.FONT_HERSHEY_SIMPLEX
            	font_scale = 1
		cv2.putText(cv_image, "Wrench location at: " + str(feature_point[0]) + "x" + str(feature_point[1]), (10, 90), font_face, font_scale, cv.RGB(0,0,255))
		cv2.putText(cv_image, "Circle's radius is: " + str(feature_point[2]) + " pixels", (10, 120), font_face, font_scale, cv.RGB(0,0,255))

                # Convert feature_point from image coordinates to world coordinates
                feature_point = np.dot(np.linalg.pinv(self.projectionMatrix), np.array([feature_point[0],feature_point[1], 1]))
                feature_point = feature_point / feature_point[2]
                feature_point = np.array((feature_point[0], feature_point[1], self.radius))
		
		# Show the real coordinates:
		cv2.putText(cv_image, "Real coordinates: " + str(feature_point[0]) + "x" + str(feature_point[1]), (10, 150), font_face, font_scale, cv.RGB(0,0,255))

                # Provide self.tracked_point to publish_poi to be published on the /poi topic
                self.tracked_point = feature_point
        except:
            pass

        return cv_image

    def get_feature_point(self, input_image):
        # Initialize key variables
	sz = np.shape(input_image)
        feature_points = list()
        feature_point = list()

        # Compute the x, y, and radius of viable circle features
        #fp = cv2.HoughCircles(input_image, **self.HoughCircles_params)
	fp = cv2.HoughCircles(input_image, cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=50, param2=30, minRadius=0, maxRadius=100)
        if fp is not None and len(fp) > 0:
            # Convert fp to a Numpy array
            feature_points = np.float32(fp).reshape(-1, 3)
	    sp = np.shape(fp)
	    nc = sp[1]
	 
	    # find the closest circle
	    dists = np.zeros(nc)
	    for i in range(nc):
    		dists[i] = np.sqrt((feature_points[i,0]-sz[1]/2)**2 + (feature_points[i,1]-sz[0]/2)**2)
    		if (dists[i] < mindist):
		    mindist = dists[i]
		    idx = i
	    feature_point = feature_points[idx]
	else:
	    rospy.loginfo("No circle found")
	#feature_point = feature_points[0]

        return feature_point

if __name__ == '__main__':
    try:
        node_name = "circle_detector"
        CircleDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down the Circle Detector node."
        cv.DestroyAllWindows()
