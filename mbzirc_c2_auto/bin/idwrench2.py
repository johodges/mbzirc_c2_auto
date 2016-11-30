#!/usr/bin/env python

""" idwrench.py - Version 1.0 2016-11-17

    This software chooses the correct wrench for the MBZIRC UGV
    challenge in an RGB image and outputs an estimate of its 3D location
    in space relative to the camera [x,y,z]
    Made by Jonathan Hodges
"""

import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
import math
from scipy.cluster import vq
import scipy.stats

class idwrench():
    def __init__(self):
        # Name this node, it must be unique
        rospy.init_node('idwrench', anonymous=True)
        
        # Enable shutdown in rospy
        rospy.on_shutdown(self.shutdown)

        # Define parameters
        self.lim_type = 0
        self.n_wr = 6
        self.segment_median_value = 3
        self.segment_area_threshold = 30
        self.segment_kernel_sz = 8
        self.save_flag = 1

        # Tweaking parameters - Need to adjust for different distances from camera
        self.area_min_thresh = 3000 # Minimum contour area to be a wrench
        self.max_circ_diam = 100 # Maximum circle diameter considered
        self.canny_param = [100, 30] # Canny edge detection thresholds
        self.p2crop = 3 # Portion of image to crop for circle detection

        self.d_mu = 22.0 # Diameter of correct wrench in pixels
        self.d_sig = 3.8 # Uncertainty in diameter
        self.l_mu = 450 # Length of correct wrench in pixels 
        self.l_sig = 25.0 # Uncertainty in length
        self.a_mu = 17500 # Area of correct wrench in pixels
        self.a_sig = 2000 # Uncertainty in area
        self.vote_wt = [0.33,0.33,0.33] # Weight of each criteria in voting (d,l,a)

        # Hardware Parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080
        # Projection matrix is from Luan
        self.projectionMatrix = np.array([(530.125732, 0.000000, 318.753955, 0.000000), (0.000000, 532.849243, 231.863630, 0.000000), (0.000000, 0.000000, 1.000000, 0.000000)])

        # Counters
        self.ct = 0

	# Establish publishers and subscribers
        self.bridge = CvBridge()
        self.id_pub = rospy.Publisher("/output/wrench_id_image",Image,queue_size = 1)
        self.binary_pub = rospy.Publisher("/output/wrench_binary_image",Image,queue_size = 1)
        self.prob_pub = rospy.Publisher("/output/wrench_prob_image",Image,queue_size = 1)
        self.probid_pub = rospy.Publisher("/output/wrench_prob_id_image",Image,queue_size = 1)
        self.image_sub = rospy.Subscriber("/mybot/camera1/image_raw",Image,self.callback)
        self.image_output = rospy.Publisher("/output/keyevent_image",Image, queue_size=1)

    # shutdown runs when this node dies
    def shutdown(self):
        rospy.sleep(0.1)

    # callback_wrench takes the RGB image and determines the (x,y,z) location of
    # the correct wrench.
    def callback(self, data):

        # This subroutine crops the RGB image to remove the gripper and the edge of
        # the box if it is visible.
        def detect_box_edge(img):
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

        # This subroutine computes the limits to be used in the brightness/contrast
        # adjustment routine. The flag denotes whether to fix the upper limit based
        # on the highest one percent of the data or not. 0 = Dynamic fix, 1 = Fixed
        # at an intensity of 255.
        def stretchlim(img,flag):
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
                if flag == 0:
                    val = 0; j = 0;
                    while val < one_perc:
                        val = val+hist[254-j]
                        j = j + 1
                    lims[i,1] = 254-j+20
                if flag == 1:
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
        def back_ground_remove(I):
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

        # This subroutine converts the RGB image without background to binary.
        # It returns the binary and grayscale images.
        def image_segmentation(img1,median_value,area_threshold,kernel_sz):
            # Convert the image to grayscale
            img2 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            # Apply 2-D median filter to the grayscale image
            img2 = cv2.medianBlur(img2,median_value)
            # Convert the grayscale image to binary using Otsu's method
            ret,final = cv2.threshold(img2, 0, 255,cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            # Find contours within the binary image
            (cnts, _) = cv2.findContours(final.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
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
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_sz,kernel_sz))
            remove = cv2.morphologyEx(final2, cv2.MORPH_OPEN, kernel)
            # Invert the image
            remove = (255-remove)
            return remove, img2

        # This subroutine detects all the circles in the image, segments them into
        # NUMBER_OF_WRENCHES bins and returns the (row,col) coordinates of the centers
        # and the mean radius of each group.
        def detect_circle(img_gray_hou):
            # Detect the circles using a hough transform
            circles = cv2.HoughCircles(img_gray_hou, cv2.cv.CV_HOUGH_GRADIENT,1,1,
                np.array([]),self.canny_param[0],self.canny_param[1],0,self.max_circ_diam)
            img_hou_all = img_gray_hou.copy()
            center_x = circles[0,:,0]
            center_y = circles[0,:,1]
            radius = circles[0,:,2]
            for n in range(len(circles[0,:,1])):
                cv2.circle(img_hou_all,(center_x[n],center_y[n]), radius[n],
                    (0,0,244), 2, cv2.CV_AA)
            cv2.imwrite('/home/jonathan/wrenchID_6_allcircles.png',img_hou_all)
            #cv2.imshow('All Circles',img_hou_all)
            #cv2.waitKey(0)
            # Establish matrix of features to use for quanitzation
            z = np.transpose(np.vstack((circles[0,:,0],circles[0,:,1])))
            # Run K-means to determine centers and to which group each point belongs.
            term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
            flag = cv2.KMEANS_RANDOM_CENTERS
            ret, labels, centers = cv2.kmeans(z, self.n_wr, term_crit, 100, flag)
            print centers
            # Find average radius within each K-means group
            rad_kmean = np.zeros([6,1])
            radius = circles[0,:,2]
            for i in range(0,6):
                locs = np.where(labels == i)
                rad = radius[locs[0]]
                print "i, rad: ", i, rad
                rad_kmean[i] = np.mean(rad)
                print "i, rad_kmean: ", i, rad_kmean[i]
            # Store center coordinates from K-means for sorting
            circs = np.zeros([6,3])
            circs[:,0:2] = centers
            circs[:,2:] = rad_kmean

            # Sort circles by x-axis
            circs = circs[circs[:,0].argsort()]
            return circs, img_hou_all

        # This subroutine determines the length and area of each segmented object
        # in the binary image. It returns length, area, and contours sorted by
        # column pixels.
        def detect_geometry(img_seg):
            # Find contours in binary image
            cnt, hie = cv2.findContours(img_seg,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            # Remove contours that are too small (noise) or too far to the left (valve)
            print "Original number of contours: ", len(cnt)
            bad_flag = 1
            while bad_flag > 0:
                bad_flag = 0
                cen2 = np.zeros([len(cnt),1])
                cnt2 = []
                cen2 = np.zeros([20,2])
                ct = 0
                for c in range(0,len(cnt)):
                    area = cv2.contourArea(cnt[c])
                    M = cv2.moments(cnt[c])
                    if area > self.area_min_thresh:
                        cen2[ct,0] = int(M["m01"] / M["m00"])
                        cen2[ct,1] = int(M["m10"] / M["m00"])
                        cnt2.append(cnt[c])
                        ct = ct+1
                cen2_mn = np.mean(cen2[0:ct-1,1])
                cen2_std = np.std(cen2[0:ct-1,1])
                cen3 = cen2[:,1]
                cen2 = cen3[np.nonzero(cen3)]
                print "STD: ", cen2_std
                if cen2_std > 300:
                    bad_flag = 1
                    ind = np.argsort(cen2)
                    cnt3 = []
                    for c in range(0,len(cnt2)):
                        if c != ind[0]:
                            cnt3.append(cnt2[c])
                    cnt = cnt3
                else:
                    cnt = cnt2
                    bad_flag = 0
            print "New number of contours: ", len(cnt)

            # Update parameters using only the good contours
            cnt2 = []; hie2 = np.zeros([6,12,4]); ct = 0
            cen2 = np.zeros([6,2]); len2 = np.zeros([6,2])
            area2 = np.zeros([6,1])
            for c in range(0,len(cnt)):
                area = cv2.contourArea(cnt[c])
                M = cv2.moments(cnt[c])
                cen2[c,1] = int(M["m01"] / M["m00"])
                cen2[c,0] = int(M["m10"] / M["m00"])
                area2[c] = area
                cnt2.append(cnt[c])
                (hi1,hi2,len2[c,0],len2[c,1]) = cv2.boundingRect(cnt[c])

            # Store all relevant features in a single matrix
            params = np.zeros([6,8])
            params[:,0:2] = cen2
            params[:,2:4] = len2
            params[:,4:5] = area2.reshape((6,1))
            # Sort contour list by x position
            ind = np.argsort(params[:,0])
            cnt2 = []
            for i in range(0,len(cnt)):
                cnt2.append(cnt[ind[i]])
            # Sort feature matrix by x position of centroid
            params = params[params[:,0].argsort()]

            return params, cnt2

        # This subroutine applies a Gaussian voting algorithm to determine which
        # wrench is the correct one using the diameter, length, and area from each
        # wrench
        def voting(params):
            votes = np.zeros([self.n_wr,3])
            votes_ideal = np.zeros([1,3])
            # Store the maximum possible probability for each parameter based
            # on the Gaussian distribution
            votes_ideal[0,0] = scipy.stats.norm(self.d_mu, self.d_sig).pdf(self.d_mu)
            votes_ideal[0,1] = scipy.stats.norm(self.l_mu, self.l_sig).pdf(self.l_mu)
            votes_ideal[0,2] = scipy.stats.norm(self.a_mu, self.a_sig).pdf(self.a_mu)
            # Compute the probability for each wrench using each parameter
            # based on the Gaussian distribution
            for i in range(0,self.n_wr):
                votes[i,0] = scipy.stats.norm(self.d_mu, self.d_sig).pdf(params[i,5])
                votes[i,1] = scipy.stats.norm(self.l_mu, self.l_sig).pdf(params[i,3])
                votes[i,2] = scipy.stats.norm(self.a_mu, self.a_sig).pdf(params[i,4])
            # Scale the probabilities based on the maximum possible for each
            # parameter
            votes = votes/votes_ideal
            vote_result = np.zeros([self.n_wr,1])
            # Sum the probabilities based on the weight values for each parameter
            vote_result = np.dot(votes,self.vote_wt)
            ind = vote_result.argsort()
            return vote_result, ind[self.n_wr-1]

        # This subroutine generates an image showing the probability that each
        # wrench is the correct one. It returns one image with all the probabilities
        # shown and one image with only the best match shown.
        def visualize_probability(img, vote_result, n, params, cnt):
            img_kmeans = img.copy()
            # Visualize the probabilities
            for i in range(self.n_wr):
                c = int(round(vote_result[i]*255))
                #cv2.circle(img_kmeans,(int(circs[i,0]),int(circs[i,1])), 
                #    int(circs[i,2]), (0,c,255-c), 2, cv2.CV_AA)
                #print "d,x,y: ", params[i,5:8]
                #print "params[i,:]: ", params[i,:]
                cv2.circle(img_kmeans,(int(params[i,6]),int(params[i,7])), 
                    int(params[i,5]), (0,c,255-c), 2, cv2.CV_AA)
                cv2.drawContours(img_kmeans, cnt[i], -1, (0,c,255-c), 3)
                #cv2.imshow('Prob',img_kmeans)
                #cv2.waitKey(0)
            img_kmeans_id = img_kmeans.copy()
            # Visualize the best match
            img_id = img.copy()
            #cv2.circle(img_id,(int(circs[n,0]),int(circs[n,1])), 
            #    int(circs[n,2]), (0,255,0), 2, cv2.CV_AA)
            cv2.circle(img_id,(int(params[n,6]),int(params[n,7])), 
                    int(params[n,5]), (0,255,0), 2, cv2.CV_AA)
            cv2.drawContours(img_id, cnt[n], -1, (0,255,0), 3)
            #cv2.circle(img_kmeans_id,(int(circs[n,0]),int(circs[n,1])), 
            #    int(circs[n,2]), (255,0,0), 2, cv2.CV_AA)
            cv2.circle(img_kmeans_id,(int(params[n,6]),int(params[n,7])), 
                    int(params[n,5]), (255,0,0), 2, cv2.CV_AA)
            cv2.drawContours(img_kmeans_id, cnt[n], -1, (255,0,0), 3)
            return img_kmeans, img_id, img_kmeans_id

        # Convert ROS image to opencv image
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            #img=cv2.imread('/home/jonathan/frame0000.jpg',1); # Image to read
            cv2.imwrite('/home/jonathan/wrenchID_0_raw.png',img)
            # Crop image to remove gripper and edge of box
            img_crop = detect_box_edge(img.copy())
            cv2.imwrite('/home/jonathan/wrenchID_1_crop.png',img_crop)
            # Determine ideal limits for brightness/contrast adjustment
            lims = stretchlim(img_crop,self.lim_type)
            # Adjust the brightness/contrast of the RGB image based on limits
            img_adj = imadjust(img_crop.copy(),lims)
            cv2.imwrite('/home/jonathan/wrenchID_2_adj.png',img_adj)
            # Remove Background from adjusted brightness/contrast image
            img_remove = back_ground_remove(img_adj.copy())
            cv2.imwrite('/home/jonathan/wrenchID_3_bg.png',img_remove)
            # Convert the image to binary
            img_seg, img_gray = image_segmentation(img_remove.copy(),
                self.segment_median_value,self.segment_area_threshold,self.segment_kernel_sz)
            sz = np.shape(img)
            cv2.imwrite('/home/jonathan/wrenchID_4_gray.png',img_gray)
            cv2.imwrite('/home/jonathan/wrenchID_5_seg.png',img_seg)
            # Crop image for circle detection
            img_gray_hou = np.copy(img_gray[0:sz[0]/self.p2crop, 0:sz[1]]) 
            # Detect circles
            circles, img_all_circles = detect_circle(img_gray_hou)
            cv2.imwrite('/home/jonathan/wrenchID_6_allcircles.png',img_all_circles)
            # Detect geometry
            params, contours = detect_geometry(img_seg)
            print "Circles: ", circles
            # Store circle diameters in feature matrix
            for i in range(0,self.n_wr):
                x_dist = abs(circles[:,0]-params[i,0])
                ind = x_dist.argsort()
                ind2 = ind[0]
                print "i: ", i
                print "All distances: ", x_dist
                print "Index of minimum distance: ", ind2
                params[i,5] = circles[ind2,2]
                params[i,6] = circles[ind2,0]
                params[i,7] = circles[ind2,1]
            #params[:,6:] = circles[:,2:]
            print params
            # Vote using the three parameters to determine correct wrench
            vote_result, wrench_ind = voting(params)
            # Visualize the probabilities and the best match
            img_kmeans, img_id, img_kmeans_id = visualize_probability(img_crop, vote_result,
                wrench_ind, params, contours)
            #cv2.imwrite('/home/jonathan/wrenchID_7_prob.png',img_kmeans)
            #cv2.imwrite('/home/jonathan/wrenchID_8_id.png',img_kmeans_id)
            # Publish results
            self.id_pub.publish(self.bridge.cv2_to_imgmsg(img_id, "bgr8"))
            self.prob_pub.publish(self.bridge.cv2_to_imgmsg(img_kmeans, "bgr8"))
            self.probid_pub.publish(self.bridge.cv2_to_imgmsg(img_kmeans_id, "bgr8"))
            self.binary_pub.publish(self.bridge.cv2_to_imgmsg(img_seg, "8UC1"))
            self.image_output.publish(self.bridge.cv2_to_imgmsg(img_kmeans_id, "bgr8"))
            ee_position = rospy.get_param('ee_position')
            wrench_position = rospy.get_param('wrench')
            xA = wrench_position[0]-ee_position[0]
            row = int(round(params[wrench_ind,7]))
            col = int(round(params[wrench_ind,6]))
            self.wrench_id_px = np.array([row,col],dtype=np.float32)
            camera_y_mx = xA*np.arctan(self.camera_fov_h/2)
            camera_y_mn = -1*xA*np.arctan(self.camera_fov_h/2)
            camera_z_mx = xA*np.arctan(self.camera_fov_v/2)
            camera_z_mn = -1*xA*np.arctan(self.camera_fov_v/2)
            # Convert the wrench pixel location to m
            print "Camera limits: ", camera_y_mx, camera_y_mn
            print "Term1: ", (1-(col/1920))
            print "Term2: ", (camera_y_mx-camera_y_mn)
            print "Term3: ", camera_y_mn
            wrenc_y = (1-(float(col)/1920))*(camera_y_mx-camera_y_mn)+camera_y_mn
            wrenc_z = (1-(float(row)/1080))*(camera_z_mx-camera_z_mn)+camera_z_mn
            self.wrench_id_m = np.array([xA, wrenc_y, wrenc_z],dtype=np.float32)
            print self.wrench_id_m
            rospy.set_param('wrench_ID',[float(self.wrench_id_px[0]), 
                float(self.wrench_id_px[1])])
            rospy.set_param('wrench_ID_m',[float(self.wrench_id_m[0]), 
                float(self.wrench_id_m[1]), float(self.wrench_id_m[2])])
            rospy.set_param('smach_state','wrenchFound')
            if self.save_flag == 1:
                cv2.imwrite('/home/jonathan/wrenchID_0_raw.png',img)
                cv2.imwrite('/home/jonathan/wrenchID_1_crop.png',img_crop)
                cv2.imwrite('/home/jonathan/wrenchID_2_adj.png',img_adj)
                cv2.imwrite('/home/jonathan/wrenchID_3_bg.png',img_remove)
                cv2.imwrite('/home/jonathan/wrenchID_4_gray.png',img_gray)
                cv2.imwrite('/home/jonathan/wrenchID_5_seg.png',img_seg)
                cv2.imwrite('/home/jonathan/wrenchID_6_allcircles.png',img_all_circles)
                cv2.imwrite('/home/jonathan/wrenchID_7_prob.png',img_kmeans)
                cv2.imwrite('/home/jonathan/wrenchID_8_id.png',img_kmeans_id)
            rospy.signal_shutdown('Ending node.')
        except:
            self.ct = self.ct+1
            if self.ct > 100:
                rospy.set_param('smach_state','wrenchNotFound')
                rospy.signal_shutdown('Ending node.')
if __name__ == '__main__':
    try:
        idwrench()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("idwrench finished.")

