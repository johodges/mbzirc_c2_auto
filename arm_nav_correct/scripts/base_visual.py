#!/usr/bin/env python
# license removed for brevity
# temperate node to publish location of wrench to test correction step
   
import rospy
import time
import cv2
import cv2.cv as cv
import sys
import actionlib
import math
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState, Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np


class BaseVisual(object):
    def __init__(self, node_name):
	self.node_name = node_name
	rospy.init_node(node_name)
	rospy.loginfo("Starting node " + str(node_name))
	rospy.on_shutdown(self.cleanup)
	rospy.Subscriber('circle', PointStamped, queue_size=10)

	# Initial the Point of interest and its Publisher
    	self.LOCS = PointStamped()
    	self.locs_pub = rospy.Publisher('/locs', PointStamped, queue_size=10)
    	self.tracked_point = None
	self.cameraMatrix = np.array([(521.186774, 0.000000, 317.250801), (0.000000, 523.451013, 235.762458), (0.000000, 0.000000, 1.000000)]) 
	self.distortion = np.array([0.132548, -0.235740, -0.009198, 0.003435, 0.000000])
	self.projectionMatrix = np.array([(530.125732, 0.000000, 318.753955, 0.000000), (0.000000, 532.849243, 231.863630, 0.000000), (0.000000, 0.000000, 1.000000, 0.000000)])
    
    def servoi(self, circle):
	try:
	    feature_point = np.dot(np.linalg.pinv(self.projectionMatrix), np.array([circle[0], circle[1], 1]))
	    feature_point = feature_point/feature_point[2]
	    dist = circle*(25/300)	# need to recalibrate
	    feature_point = np.array((feature_point[0], feature_point[1], dist))
	    self.tracked_point = feature_point
	except:
	    pass
#	return:
#	    cv_image

    def publish_locs(self):
	if self.tracket_point is not None and len(self.tracked_point) > 0:
	    locs = self.tracked_point
    	else:
	    return
	try:
	    self.LOCS = PointStamped()
    	    self.LOCS.header.frame_id = "camera_fram_id"
    	    self.LOCS.header.stamp = rospy.Time.now()
    	    self.LOCS.point.x = float(locs[0])
    	    self.LOCS.point.y = float(locs[1])
   	    self.LOCS.point.z = float(locs[2])
	    self.locs_pub.publish(self.LOCS)
	except:
	    rospy.loginfo("Unable to publishing LOCATIONS!")
     
    def cleanup(self):
	print "Shuting downt the Vision base node. "
	cv2.destroyAllWindows()

def main(args):
    try:
	node_name="BaseVisual"
	BaseVisual(node_name)
	rospy.spin()
    except KeyboardInterrupt:
	print "Shutting down Base Visual node."
	cv2.destroyAllWindows()
	

if __name__ == '__main__':
    main(sys.argv)

