#!/usr/bin/env python

""" wrench.py - version 1.0 2016-11-10

    This node will detect the closest circle regard the center point of image frame
    Point out the 3D coordinate and the modified distances
    
    LUAN CONG DOAN
    CMS Lab
"""

import rospy
import rospkg
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, PointStamped
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
import math
import sys
import roslib



class wrench_vision:
    def __init__(self):
	# Define parameters
	self.cam_foh = 1.5708
	self.cam_fov = 1.5708
	self.cam_h = 1920
	self.cam_v = 1080
	self.tracked_point = np.zeros(3)
	# Camera info from real camera
        #self.cameraMatrix = np.array([(521.186774, 0.000000, 317.250801), (0.000000, 523.451013, 235.762458), (0.000000, 0.000000, 1.000000)]) 
	#self.distCoeffs = np.array([0.132548, -0.235740, -0.009198, 0.003435, 0.000000])
	#self.projectionMatrix = np.array([(530.125732, 0.000000, 318.753955, 0.000000), (0.000000, 532.849243, 231.863630, 0.000000), (0.000000, 0.000000, 1.000000, 0.000000)])
	# Camera info from Simulation - not sure
	self.projectionMatrix = np.array([(959.9964737295771, 0.0, 960.5, -67.1997531610704), (0.0, 959.9964737295771, 540.5, 0.0), (0.0, 0.0, 1.0, 0.0)])
	
	# Establish publishers and subscribers
	self.bridge = CvBridge()
	self.image_pub = rospy.Publisher("/output_wrench_vision", Image, queue_size=10)
	self.image_sub = rospy.Subscriber("/input_arm_cam", Image, self.callback)
	
	# Initialize Location
	self.LOC = PointStamped()
	self.loc_pub = rospy.Publisher("/loc", PointStamped, queue_size=5)
    
    
    def callback(self,data):
	try:	
	    img = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
	    print e
	
	if img is not None:
	    # get size of image
	    sz = np.shape(img)
	    # print sz

	    # test subscribe and publisher - work fine
	    #if sz[0] > 60 and sz[1] > 60 :
            #    cv2.circle(img, (200,200), 500, 255, 10)


	    # applied filter Blur
	    img = cv2.medianBlur(img,5)
	    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	    # detect all posible circles
	    circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=50, param2=30, minRadius=30, maxRadius=100)
	    #for i in circles[0,:]:
    		#cv2.circle(img, (i[0],i[1]), i[2], (0,255,0), 2)
    		#cv2.circle(img, (i[0],i[1]), 2, (0,0,255), 3)
    	    cv2.circle(img, (sz[1]/2,sz[0]/2), 2, (255,0,0), 3)
	    if circles is not None:
		# get number of detected circles
		sp = np.shape(circles)
		nc = sp[1]
		rospy.loginfo("Circle found!")
		#print nc	# call out number of detected circle

		# find the closest circles regarding center point
		data = np.uint16(np.around(circles)).reshape(-1,3)
		dists = np.zeros(nc)
		mindist = 2000
		idx = 0
		for i in range(nc):
   		    dists[i] = np.sqrt((data[i,0]-sz[1]/2)**2 + (data[i,1]-sz[0]/2)**2)
  		    if (dists[i] < mindist):
			mindist = dists[i]
			idx = i
		print mindist
		print idx
		# get the best circle
		best = data[idx]
		print best
		
		# Draw selected circle and point out some information
		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(img, 'Image size is: ' + str(sz[0]) + "x" + str(sz[1]),  (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0)) 
		cv2.putText(img, 'Number of detected circles: ' + str(nc),  (10,80), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0)) 
		cv2.putText(img, 'The selected wrench position is: ' + str(best[0]) + "x" + str(best[1]),  (10,130), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0)) 
#		cv2.putText(img, 'Circle diameter is: ' + str(best[2]),  (10,120), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,0))
		cv2.circle(img, (best[0],best[1]), 2, (0,0,255), 3)
		cv2.circle(img, (best[0],best[1]), best[2], (0,255,0), 2)

		# Moving distances:
		x = (0.014*self.cam_foh)/best[2]
		y_pix = best[0] - sz[1]/2
		z_pix = best[1] - sz[0]/2
		feature_point = [x,y_pix,z_pix]
		
		feature_point = np.dot(np.linalg.pinv(self.projectionMatrix), np.array([feature_point[1],feature_point[2], 1]))
            	feature_point = feature_point / (1000*feature_point[2])
            	self.tracked_point = np.array((x, feature_point[0], feature_point[1]))

		print self.tracked_point 
#		rospy.set_param('smach_state','Wrench location detected')
#		rospy.signal_shutdown('Ending node')
		
		# publish location
		self.publish_loc()
	    else:
		rospy.loginfo("No circle found")
#		rospy.set_param('smach_state', 'Wrench location NOT FOUND!')
	else:
	    rospy.loginfo('No image input')

	# Print out image
	cv2.namedWindow('Output Wrench Vision', cv2.WINDOW_NORMAL)		
	cv2.imshow('Output Wrench Vision', img)
	cv2.waitKey(3)
	    
	try:
	    self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
	except CvBridgeError as e:
	    print (e)

    def publish_loc(self):
	if self.tracked_point is not None and len(self.tracked_point) > 0:
	    loc = self.tracked_point
	else:
	    return
	try:
	    self.LOC = PointStamped()
	    self.LOC.header.frame_id = "camera_frame_id"
	    self.LOC.header.stamp = rospy.Time.now()
	    self.LOC.point.x = loc[0]
	    self.LOC.point.y = loc[1]
	    self.LOC.point.z = loc[2]
	    self.loc_pub.publish(self.LOC)
	except:
	    rospy.loginfo("Failed to publishing LOCATION")

def main(args):
    ic = wrench_vision()
    rospy.init_node('wrench_vision', anonymous=True)
    try:
	rospy.spin()
    except KeyboardInterrupt:
	print("Shuting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



