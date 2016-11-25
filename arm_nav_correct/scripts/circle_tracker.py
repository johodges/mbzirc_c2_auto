#!/usr/bin/env python

""" circle_tracker.py - Version 1.0 2016-02-23

    Based on the R. Patrick Goebel's lk_tracker.py demo code

    Tracks the HoughCircles feature closest to the center of the image in a video stream

    Created for the CCAM Project

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
from circle_detector import CircleDetector

class CircleTracker(CircleDetector):
    def __init__(self, node_name):
        super(CircleTracker, self).__init__(node_name)

        #Do we show text on the display?
        self.show_text = rospy.get_param("~show_text", True)

        # How big should the feature points be (in pixels)?
        self.feature_size = rospy.get_param("~feature_size", 1)

        # How often should we refresh the feature
        self.drop_feature_point_interval = rospy.get_param("~drop_feature_point_interval", 8)

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

        # LK parameters
        self.lk_winSize = rospy.get_param("~lk_winSize", (10, 10))
        self.lk_maxLevel = rospy.get_param("~lk_maxLevel", 2)
        self.lk_criteria = rospy.get_param("~lk_criteria", (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01))

        # Store all the LKT tracker parameters together for passing to tracker
        self.lk_params = dict( winSize  = self.lk_winSize,
                  maxLevel = self.lk_maxLevel,
                  criteria = self.lk_criteria)

        # Initialize key variables
        self.feature_point = None
        self.tracked_point = None
        self.mask = None
        self.grey = None
        self.prev_grey = None
        self.frame_index = 0

    def process_image(self, cv_image):
        try:
            #Step 1: Image processing
            # Mask image
            self.mask = np.zeros_like(cv_image)
            x, y, w, h = self.frame_width / 4, self.frame_height / 4, self.frame_width / 2, self.frame_height / 2
            self.mask[y:y+h, x:x+w] = 255
            masked_image = cv2.bitwise_and(cv_image, self.mask)

            # Create a grey scale version of the image
            self.grey = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

            # Equalize the histogram to reduce lighting effects
            self.grey = cv2.equalizeHist(self.grey)

            # Remove salt-and-pepper, and white noise by using a combination of a median and Gaussian filter
            self.grey = cv2.medianBlur(self.grey, **self.medianBlur_params)
            self.grey = cv2.GaussianBlur(self.grey, **self.GaussianBlur_params)

            #Step 2: Extracting feature_points
            # If we haven't yet started tracking a feature point, extract a feature point from the image
            if self.tracked_point is None:
                self.feature_point = self.get_feature_point(self.grey)

            #Step 3: If we have a feature points, track it using optical flow
            if len(self.feature_point) > 0:
                # Store a copy of the current grey image used for LK tracking
                if self.prev_grey is None:
                    self.prev_grey = self.grey

                # Now that have a feature point, track it to the next frame using optical flow
                self.tracked_point = self.track_feature_point(self.grey, self.prev_grey)
            else:
                # We have lost all feature_points so re-detect circle feature
                self.tracked_point = None

            #Step 4: Re-dectect circle feature every N frames
            #if self.frame_index % self.drop_feature_point_interval == 0 and len(self.feature_point) > 0:
            #    self.tracked_point = None
            #    self.frame_index = 0

            #self.frame_index += 1


            # Process any special keyboard commands for this module
            if self.keystroke != -1:
                try:
                    cc = chr(self.keystroke & 255).lower()
                    if cc == 'c':
                        # Clear the current feature_points
                        self.feature_point = None
                        self.tracked_point = None
                except:
                    pass

            self.prev_grey = self.grey
        except:
            pass

        return cv_image

    def track_feature_point(self, grey, prev_grey):
        # We are tracking points between the previous frame and the current frame
        img0, img1 = prev_grey, grey

        # Reshape the current feature point into a Numpy array required by calcOpticalFlowPyrLK()
        p0 = self.feature_point[:2].reshape(-1, 1 ,2)

        # Calculate the optical flow from the previous frame to the current frame
        p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)

        # Do the reverse calculation: from the current frame to the previous frame
        try:
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)

            # Compute the distance between corresponding points in the two flows
            d = abs(p0-p0r).reshape(-1, 2).max(-1)

            # If the distance between pairs of points is < 1 pixel, set
            # a value in the "good" array to True, otherwise False
            good = d < 1

            # Initialize a list to hold new feature_points


            # Cycle through all current and new feature_points and only keep
            # those that satisfy the "good" condition above
            for (x, y), good_flag in zip(p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                new_feature_point = np.array((x, y, self.feature_point[2]))

            # Draw the center of the circle
            cv2.circle(self.marker_image, (new_feature_point[0], new_feature_point[1]), self.feature_size, (0, 0, 255, 0), cv.CV_FILLED, 8, 0)
            # Draw the outer circle
            cv2.circle(self.marker_image, (new_feature_point[0], new_feature_point[1]), new_feature_point[2], (0, 255, 0, 0), self.feature_size, 8, 0)

            # Set the global feature_point list to the new list
            self.feature_point = new_feature_point

            # Convert feature_point from image coordinates to world coordinates
            world_coordinates = np.dot(np.linalg.pinv(self.projectionMatrix), np.array([self.feature_point[0],self.feature_point[1], 1]))
            world_coordinates = world_coordinates / world_coordinates[2]
            world_coordinates = np.array((world_coordinates[0], world_coordinates[1]))

            # Provide self.tracked_point to publish_poi to be published on the /poi topic
            self.tracked_point = world_coordinates

        except:
            self.tracked_point = None

        return self.tracked_point

if __name__ == '__main__':
    try:
        node_name = "circle_tracker"
        CircleTracker(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Circle Tracking node."
        cv.DestroyAllWindows()
