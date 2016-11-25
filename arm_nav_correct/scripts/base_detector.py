#!/usr/bin/env python

""" BaseDetector.py - Version 1.1 2016-05-24

    Based on the R. Patrick Goebel's ros2opencv2.py demo code

    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).

    Includes variables and helper functions to store detection and tracking information and display
    markers on the image.

    Creates an POI publisher to publish the region of interest on the /poi topic.

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
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np

class BaseDetector(object):
    def __init__(self, node_name):
        self.node_name = node_name

        rospy.init_node(node_name)

        rospy.loginfo("Starting node " + str(node_name))

        rospy.on_shutdown(self.cleanup)

        # A number of parameters to determine what gets displayed on the
        # screen. These can be overridden the appropriate launch file
        self.show_text = rospy.get_param("~show_text", True)
        self.flip_image = rospy.get_param("~flip_image", False)
        self.feature_size = rospy.get_param("~feature_size", 1)

        # Initialize the Point of Interest and its publisher
        self.POI = PointStamped()
        self.poi_pub = rospy.Publisher("/poi", PointStamped, queue_size=5)

        # Initialize a number of global variables
        self.frame = None
        self.frame_size = None
        self.frame_width = None
        self.frame_height = None
        self.feature_point = list()
        self.mask = None
        self.depth_image = None
        self.marker_image = None
        self.display_image = None
        self.grey = None
        self.prev_grey = None
        self.tracked_point = None
        self.keystroke = None
        self.keep_marker_history = False
        self.night_mode = False
        self.cps = 0 # Cycles per second = number of processing loops per second.
        self.cps_values = list()
        self.cps_n_values = 20
        self.resize_window_width = 0
        self.resize_window_height = 0
        self.undistort_image = True
	# Camera info from real camera
        #self.cameraMatrix = np.array([(521.186774, 0.000000, 317.250801), (0.000000, 523.451013, 235.762458), (0.000000, 0.000000, 1.000000)]) 
	#self.distCoeffs = np.array([0.132548, -0.235740, -0.009198, 0.003435, 0.000000])
	#self.projectionMatrix = np.array([(530.125732, 0.000000, 318.753955, 0.000000), (0.000000, 532.849243, 231.863630, 0.000000), (0.000000, 0.000000, 1.000000, 0.000000)])
	# Camera info from Simulation
	self.projectionMatrix = np.array([(959.9964737295771, 0.0, 960.5, -67.1997531610704), (0.0, 959.9964737295771, 540.5, 0.0), (0.0, 0.0, 1.0, 0.0)])

        # Create the main display window
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        if self.resize_window_height > 0 and self.resize_window_width > 0:
            cv.ResizeWindow(self.cv_window_name, self.resize_window_width, self.resize_window_height)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the image and depth topics and set the appropriate callbacks
        # The image topic names can be remapped in the appropriate launch file
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=5)
        #self.depth_sub = rospy.Subscriber("input_depth_image", Image, self.depth_callback, queue_size=5)

        # Create an image publisher to output the display_image
        self.image_pub = rospy.Publisher("output_rgb_image", Image, queue_size=5)

    def image_callback(self, data):
        # Store the image header in a global variable
        self.image_header = data.header

        # Time this loop to get cycles per second
        start = rospy.get_time()

        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        frame = self.convert_image(data)

        # Some webcams invert the image
        if self.flip_image:
            frame = cv2.flip(frame, 0)

        # Undistort image if necessary
        #if self.undistort_image:
        #    frame = cv2.undistort(frame, self.cameraMatrix, self.distCoeffs)

        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Create the marker image we will use for display purposes
        if self.marker_image is None:
            self.marker_image = np.zeros_like(frame)

        # Copy the current frame to the global image in case we need it elsewhere
        self.frame = frame.copy()

        # Reset the marker image if we're not displaying the history
        if not self.keep_marker_history:
            self.marker_image = np.zeros_like(self.marker_image)

        # Process the image to detect and track objects or features
        processed_image = self.process_image(frame)

        # Make a global copy
        self.processed_image = processed_image.copy()

        # Night mode: only display the markers
        if self.night_mode:
            self.processed_image = np.zeros_like(self.processed_image)

        # Merge the processed image and the marker image
        self.display_image = cv2.bitwise_or(self.processed_image, self.marker_image)

        # Publish the POI
        self.publish_poi()

        # Handle keyboard events
        self.keystroke = cv.WaitKey(5)

        # Compute the time for this loop and estimate CPS as a running average
        end = rospy.get_time()
        duration = end - start
        fps = int(1.0 / duration)
        self.cps_values.append(fps)
        if len(self.cps_values) > self.cps_n_values:
            self.cps_values.pop(0)
        self.cps = int(sum(self.cps_values) / len(self.cps_values))

        # Display CPS and image resolution if asked to
        if self.show_text:
            font_face = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1

            """ Print cycles per second (CPS) and resolution (RES) at top of the image """
            if self.frame_size[0] >= 640:
                vstart = 25
                voffset = int(50 + self.frame_size[1] / 120.)
            elif self.frame_size[0] == 320:
                vstart = 15
                voffset = int(35 + self.frame_size[1] / 120.)
            else:
                vstart = 10
                voffset = int(20 + self.frame_size[1] / 120.)
            cv2.putText(self.display_image, "CPS: " + str(self.cps), (10, vstart), font_face, font_scale, cv.RGB(0, 0, 255))
            cv2.putText(self.display_image, "RES: " + str(self.frame_size[0]) + "X" + str(self.frame_size[1]), (10, voffset), font_face, font_scale, cv.RGB(0, 0, 255))
	    

        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if self.keystroke != -1:
            try:
                cc = chr(self.keystroke & 255).lower()
                if cc == 'n':
                    self.night_mode = not self.night_mode
                elif cc == 't':
                    self.show_text = not self.show_text
                elif cc == 'q':
                    # The has press the q key, so exit
                    rospy.signal_shutdown("User hit q key to quit.")
            except:
                pass

        # Publish the display image
        self.publish_display_image(self.display_image)

        # Update the image display
        cv2.imshow(self.node_name, self.display_image)


    def depth_callback(self, data):
        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        depth_image = self.convert_depth_image(data)

        # Some webcams invert the image
        if self.flip_image:
            depth_image = cv2.flip(depth_image, 0)

        # Process the depth image
        processed_depth_image = self.process_depth_image(depth_image)

        # Make global copies
        self.depth_image = depth_image.copy()
        self.processed_depth_image = processed_depth_image.copy()

    def convert_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

    def convert_depth_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")

            # Convert to a numpy array since this is what OpenCV uses
            depth_image = np.array(depth_image, dtype=np.float32)

            return depth_image

        except CvBridgeError, e:
            print e

    def publish_display_image(self, display_image):
        if display_image is not None:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(display_image, 'bgr8'))

            except CvBridgeError, e:
                print e
        else:
            return

    def publish_poi(self):
        if self.tracked_point is not None and len(self.tracked_point) > 0:
            poi = self.tracked_point
        else:
            return

        try:
            self.POI = PointStamped()
            self.POI.header.frame_id = "camera_frame_id"
            self.POI.header.stamp = rospy.Time.now()
            self.POI.point.x = poi[0]
            self.POI.point.y = poi[1]
            self.POI.point.z = poi[2]
            self.poi_pub.publish(self.POI)
        except:
            rospy.loginfo("Publishing POI failed")

    def process_image(self, frame):
        return frame

    def process_depth_image(self, frame):
        return frame

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):
    try:
        node_name = "BaseDetector"
        BaseDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down BaseDetector node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
