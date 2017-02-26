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
import rospkg
import moveit_commander
from moveit_msgs.msg import RobotState
from control_msgs.msg import FollowJointTrajectoryActionResult
import sys
from sensor_msgs.msg import Image
import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import math

class valve_orient():
    def __init__(self):
        # Name this node, it must be unique
        rospy.init_node('valve_orient', anonymous=True, log_level=rospy.DEBUG)

        # Enable shutdown in rospy (This is important so we cancel any move_base goals
        # when the node is killed)
        rospy.on_shutdown(self.shutdown) # Set rospy to execute a shutdown function when exiting
        rospack = rospkg.RosPack()  # Find rospackge locations

        # Store camera parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080
        self.ct5 = 0
        self.lim_type = 0
        self.xA = 0.380
        self.canny_param = [10, 60] # Canny edge detection thresholds
        self.save_result = False
        self.preview_flag = False           # Should we preview images
        self.preview_result = False
        self.indir = str(rospack.get_path('mbzirc_c2_auto')+'/params/')
        self.lim_adjust = 100
        rospy.set_param('wrench_mask_file','/home/jonathan/mask_image.jpg')
        rospy.sleep(0.1)
        try:
            wrench_mask_file = rospy.get_param('wrench_mask_file')
        except:
            pass
        #rospy.set_param('valve',[0.3,0.2,0.2])
        try:
            self.wrench_slope = rospy.get_param('wrench_slope')
        except:
            rospy.set_param('wrench_slope',[-0.35,-0.35,-0.35])
            rospy.sleep(0.1)
            self.wrench_slope = rospy.get_param('wrench_slope')

        # Set up ROS subscriber callback routines
        self.bridge = CvBridge()
        self.wrench_mask = cv2.imread(wrench_mask_file)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.image_valve_stem = rospy.Publisher("/output/valve_stem",Image, queue_size=1)
        self.image_output = rospy.Publisher("/output/keyevent_image",Image, queue_size=1)
        
        """
        # Set up moveit joint control
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander("ur5_arm")

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo(self.end_effector_link)

        # Initialize Necessary Variables
        reference_frame = rospy.get_param("~reference_frame", "/base_link")

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.0001)
        self.arm.set_goal_orientation_tolerance(0.001)

        # Set the target pose from the input
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = reference_frame
        """

    def shutdown(self):
        rospy.sleep(1)

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
            ret,labels,centers = cv2.kmeans(z,2,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
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
        
        #cv2.imwrite('%sidvalve_rgb_%s_%s.png' % (self.indir, str(int(1000*self.xA)), str(img_count)),cv_image)

        cimg = cv2.medianBlur(cv_image,5)
        lims = stretchlim(cimg)
        print lims
        img_invert = cimg.copy()
        img_invert[self.wrench_mask == 255] = 0
        if self.preview_flag:
            cv2.imshow('img_invert',img_invert)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        cent_x = 1920/2-50
        cent_y = 1080/2+10
        box_width = 100

        box_img = img_invert[cent_y-box_width:cent_y+box_width,cent_x-box_width:cent_x+box_width].copy()
        if self.preview_flag:
            cv2.imshow('box_img',box_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        box_img_hsv = cv2.cvtColor(box_img.copy(), cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(box_img, cv2.COLOR_BGR2GRAY)
        if self.preview_flag:
            cv2.imshow('hue',box_img_hsv[:,:,0])
            cv2.imshow('sat',box_img_hsv[:,:,1])
            cv2.imshow('val',box_img_hsv[:,:,2])
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        img_gray[box_img_hsv[:,:,1] > 100] = 0
        img_rgb = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
        if self.preview_flag:
            cv2.imshow('box_img',img_rgb)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        lims = stretchlim(img_rgb)
        img_adj = imadjust(img_rgb.copy(),lims)
        if self.preview_flag:
            cv2.imshow('img_adj',img_adj)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        img_remove = back_ground_remove(img_adj.copy(),img_rgb.copy())
        img_remove[img_remove > 0] = 255
        if self.preview_flag:
            cv2.imshow('img_remove',img_remove)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        kernel1 = np.ones((3,3), np.uint8)
        img_edge1 = cv2.erode(img_remove, kernel1, iterations=5)
        if self.preview_flag:
            cv2.imshow('img_edge',img_edge1)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        img_edge2 = cv2.dilate(img_edge1, kernel1, iterations=5)
        if self.preview_flag:
            cv2.imshow('img_edge',img_edge2)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        im_floodfill = img_edge2.copy()
        h, w = img_edge2.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(im_floodfill, mask, (0,0), 255);

        # Invert floodfilled image
        im_floodfill_inv = cv2.bitwise_not(im_floodfill)

        # Combine the two images to get the foreground.
        im_out = img_edge2 | im_floodfill_inv

        #img_remove_gray = cv2.cvtColor(img_invert, cv2.COLOR_BGR2GRAY)
        img_edge = cv2.Canny(im_out,self.canny_param[0],self.canny_param[1])
        if self.preview_flag:
            cv2.imshow('img_edge',img_edge)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        lines = cv2.HoughLines(img_edge,1,np.pi/180,25)
        sz = np.shape(img_edge); vert_line1 = sz[1]/2; vert_line2 = sz[1]/2;
        img_edge = cv2.cvtColor(img_edge.copy(), cv2.COLOR_GRAY2BGR)
        ln_sz = np.shape(lines)
        m = np.zeros(ln_sz[1],)
        ct = 0
        for rho,theta in lines[0]:
            #if abs(theta) > 2.80 and abs(theta) < 3.14:
            a = np.cos(theta); b = np.sin(theta);
            x0 = a*rho; y0 = b*rho
            x1 = float(x0 + 1000*(-b)); y1 = float(y0 + 1000*(a));
            x2 = float(x0 - 1000*(-b)); y2 = float(y0 - 1000*(a));
            try:
                m[ct] = (y2-y1)/(x2-x1); b = y1-m*x1;
                ybot = sz[0]; xbot = (ybot-b)/m[ct]
                #if xbot < vert_line1:
                #    vert_line1 = xbot; m1 = m; b1 = b; x1_1 = x1; y1_1 = y1;
                #if xbot > vert_line2:
                #    vert_line2 = xbot; m2 = m; b2 = b; x2_1 = x1; y2_1 = y1;
                cv2.line(box_img,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),2)
                ct = ct+1
            except:
                pass
        term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
        flag = cv2.KMEANS_RANDOM_CENTERS
        z = np.float32(m[0:ct].reshape((-1,1)))
        ret2, labels, centers = cv2.kmeans(z, 2, term_crit, 10, flag)
        print "CLUSTERED STRONGEST 2 SLOPES: ", centers
        dist = np.power(np.power(centers-self.wrench_slope[2],2),0.5)
        valve_angle = centers[np.argmin(dist)]
        dist2rotate = self.wrench_slope[2]-valve_angle
        print "location of minimum distance: ", np.argmin(dist)
        print "closest valve angle to wrench: ", valve_angle
        print "wrench angle: ", self.wrench_slope[2]
        print "radians to rotate end effector: ", dist2rotate
        if self.preview_result:
            cv2.imshow('img',box_img)
            cv2.waitKey(0)
        rospy.sleep(0.1)
        if min(dist) < 0.017:
            print "Valve and wrench are within 1 degree!"
            tmp = box_img[:,:,2]
            box_img[tmp == 255,1] = 255
            box_img[tmp == 255,2] = 0             
            self.image_valve_stem.publish(self.bridge.cv2_to_imgmsg(box_img, "bgr8"))
            rospy.sleep(0.1)
            rospy.signal_shutdown('Ending node.')
        else:
            print "Rotating the end effector: ", dist2rotate
            self.image_valve_stem.publish(self.bridge.cv2_to_imgmsg(box_img, "bgr8"))
            rospy.sleep(0.1)
            """
            # Set the start state to the current state
            self.arm.set_start_state_to_current_state()
            self.current_wrist_joint = self.arm.get_joint_value('ur5_arm_wrist_3_joint')
            self.arm.set_joint_value_target('ur5_arm_wrist_3_joint', self.current_wrist_joint+dist2rotate)
            traj = self.arm.plan()
            self.arm.execute(traj)
            """

        if self.preview_flag:
            cv2.imshow('img',img_edge)
            cv2.waitKey(0)

if __name__ == '__main__':
    try:
        valve_orient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("valve_orient finished.")

