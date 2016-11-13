#!/usr/bin/env python

""" idwrench.py - Version 1.0 2016-10-12

    This software chooses the left most wrench in an RGB image and outputs an
    estimate of its 3D location in space relative to the camera [x,y,z]
    Made by Jonathan Hodges

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
import rospkg
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from rospy_tutorials.msg import Floats
import numpy as np
from decimal import *
import tf
import math

class idwrench():
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('idwrench', anonymous=True)
        
        # Enable shutdown in rospy (This is important so we cancel any move_base goals
        # when the node is killed)
        rospy.on_shutdown(self.shutdown) # Set rospy to execute a shutdown function when exiting
        self.ct = 0

        # Store camera parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080
	# convert image from ROS to OpenCV
	self.image_pub = rospy.Publisher("image_1",Image,queue_size = 20)
 
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("/mybot/camera1/image_raw",Image,self.callback)

        # Set up ROS subscriber callback routines+
        #rospy.Subscriber("/wrench_centroids", numpy_msg(Floats), self.callback_wrench, queue_size=1)
	# convert image from ROS to OpenCV

    def shutdown(self):
        rospy.sleep(1)

    # callback_wrench is used to store the wrench topic into the class to be
    # referenced by the other callback routines.
    def callback(self, data):

        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Detection code starts here
        b = img[:,:,0]
        g = img[:,:,1]
        r = img[:,:,2]
	
        histB,binsB = np.histogram(b.ravel(),256,[0,256])
        histG,binsG = np.histogram(g.ravel(),256,[0,256])
        histR,binsR = np.histogram(r.ravel(),256,[0,256])

        countB, _, _ = plt.hist(histB,binsB)
        mB =  np.max(countB)
        countG, _, _ = plt.hist(histG,binsG)
        mG = np.max(countG)
        countR, _, _ = plt.hist(histR,binsR)
        mR = np.max(countR)

        b1 = b + mB
        g1 = g + mG
        r1 = r + mR

        img1 = img.copy()
        img1[:,:,0] = b1
        img1[:,:,1] = g1
        img1[:,:,2] = r1

        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(img1,105,255,0)
        contours,hierarchy = cv2.findContours(thresh, 1, 2)
        scale_factor = 1
        L = []
        A = []
        R = []
        xL = []
        xA = []
        xR = []
        yR = []
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            if h > 300  and h < 1000:
                epsilon = 0.1*cv2.arcLength(cnt,True)
                approx = cv2.approxPolyDP(cnt,epsilon,True)
                area = cv2.contourArea(cnt)
                A.append(area)

                cv2.drawContours(img, cnt, -1, (0,255,0), 3)
                L.append(h)
                xL.append(x + w/2)
                crop = img1[y-20:y+h/5,x:x+w]
                cv2.rectangle(img,(x,y),(x+w,y+h),(100,255,0),2)
                crop = cv2.resize(crop,(0,0),fx=scale_factor,fy=scale_factor)
                crop1 = cv2.medianBlur(crop,5)
	
                circles = cv2.HoughCircles(crop1, cv2.cv.CV_HOUGH_GRADIENT,1,10,np.array([]),100,38,10,40)
                if circles != None:
                    a,b,c = circles.shape
                    for i in range(b):
                        cv2.circle(crop,(circles[0][i][0], circles[0][i][1]), circles[0][i][2], (255,0,0), 1, cv2.CV_AA)
                        cv2.circle(crop,(circles[0][i][0], circles[0][i][1]), 2, (255,100,0), 1, cv2.CV_AA)
                        x0 = int(circles[0][i][0]/scale_factor)
                        y0 = int(circles[0][i][1]/scale_factor)
                        # get the size of the circles
                        R.append(circles[0][i][2]/scale_factor)
                        xR.append(x0+x)
                        yR.append(y0+y-20)
                        #print circles[0][i][2]/scale_factor
                        cv2.circle(img,(x0+x, y0+y-20), int(circles[0][i][2]/scale_factor), (255,0,0), 2, cv2.CV_AA)
                        cv2.circle(img,(x0+x, y0+y-20), 2, (255,100,0), 1, cv2.CV_AA)
        A = np.array(A)
        L = np.array(L)
        R = np.array(R)
        xL = np.array(xL)
        xR = np.array(xR)
        yR = np.array(yR)

        sA = np.sort(A)
        sL = np.sort(L)
        sR =np.sort(R)

        indexA = np.argsort(A)
        indexR = np.argsort(R)
        indexL = np.argsort(L)
        x_L = np.zeros((np.size(xL,0)))
        x_R = np.zeros((np.size(xR,0)))

        for i in range(np.size(indexR)):
            x_R[i] = xR[indexR[i]]	
        for i in range(np.size(indexL)):
            x_L[i] = xL[indexL[i]]

        vote = np.zeros((np.size(x_L,0),1))
        vote[indexL[2]] = vote[indexL[2]] + 0.5
        vote[indexA[2]] = vote[indexA[2]] + 0.3
        vote[indexR[2]] = vote[indexR[2]] + 0.2

        index = np.argmax(vote)
        wrench = np.array([xR[index],yR[index]])
        print wrench
        if wrench.size:
            xA = 0.6
            print "ID wrench in pixels: ", wrench
            camera_y_mx = xA*np.tan(self.camera_fov_h/2)
            camera_y_mn = -1*xA*np.tan(self.camera_fov_h/2)
            camera_z_mx = xA*np.tan(self.camera_fov_v/2)
            camera_z_mn = -1*xA*np.tan(self.camera_fov_v/2)
            print "Camera ymn/ymx: ", camera_y_mn, camera_y_mx
            wrenc_y = (wrench[1]-1080)/(2160-0)*(camera_y_mx-camera_y_mn)+camera_y_mn
            wrenc_z = (wrench[0]-1920)/(3840-0)*(camera_z_mx-camera_z_mn)+camera_z_mn
            self.wrench_id = np.array([xA, wrenc_y, wrenc_z],dtype=np.float32)
            print "ID wrench in m: ", self.wrench_id
            rospy.set_param('wrench_ID',[float(self.wrench_id[0]), float(self.wrench_id[1]), float(self.wrench_id[2])]) 
            rospy.set_param('smach_state','wrenchFound')
            rospy.signal_shutdown('Ending node.')
        else:
            self.ct = self.ct+1
            if self.ct > 100:
                rospy.set_param('smach_state','wrenchNotFound')
                rospy.signal_shutdown('Ending node.')
            rospy.sleep(0.1)
if __name__ == '__main__':
    try:
        idwrench()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("idwrench finished.")

