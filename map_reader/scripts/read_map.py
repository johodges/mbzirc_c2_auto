#!/usr/bin/env python

import rospy
import yaml
import cv
import cv2
import sys
from std_msgs.msg import String
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import tf

def main():
	rospy.init_node('arena_map')

	listener = tf.TransformListener()

	#im = cv2.imread('/home/tom/catkin_ws/src/mbzirc_c2_auto/map_reader/arenamap.pgm')

	with open("/home/tom/catkin_ws/src/mbzirc_c2_auto/map_reader/mapCoords.yaml", 'r') as f:
    		doc = yaml.load(f)

	f.close()

	print doc["metersToPixels"]
	rospy.set_param("/metersToPixels", doc["metersToPixels"])

	initialX = doc["startingPoint"][0]
	initialY = doc["startingPoint"][1]

	rospy.set_param("/initialX", initialX)
	rospy.set_param("/initialY", initialY)

	roboX = initialX
	roboY = initialY
	print roboX
	print roboY

	arenaPnt1 = doc["arenaTopLeft"]
	arenaPnt2 = doc["arenaBotRight"]

	rospy.set_param("/arenaPnt1", arenaPnt1)
	rospy.set_param("/arenaPnt2", arenaPnt2)

	badSquare1 = doc["deadZoneTopLeft"]
	badSquare2 = doc["deadZoneBotRight"]

	rospy.set_param("/deadZone1", badSquare1)
	rospy.set_param("/deadZone2", badSquare2)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans, rotate) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "Didn't work"
			continue
		print listener.lookupTransform('/map', '/odom', rospy.Time(0))
		try:
			(trans, rotate) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		#print rotate[2]
		rospy.set_param("/currentRobotAngle", rotate[2])
		
		roboX = initialX + (trans[0] * 24)
		roboY = initialY - (trans[1] * 24)
		#print roboX
		#print roboY

		rospy.set_param("/currentRobotX", roboX)
		rospy.set_param("/currentRobotY", roboY)

		#if roboX > arenaPnt1[0] and roboX < arenaPnt2[0] and roboY > arenaPnt1[1] and roboY < arenaPnt2[1]:
			#print "Inside the arena"
		#else:
			#print "Outside the arena"

		#if roboX > badSquare1[0] and roboX < badSquare2[0] and roboY > badSquare1[1] and roboY < badSquare2[1]:
			#print "Should not be here"

		rate.sleep()

	#cv_window_name = "Map"

	#cv.NamedWindow(cv_window_name, 0)

	#cv2.circle(im, (int(roboX), int(roboY)), 5, (255, 0, 0), -1)
	#cv2.rectangle(im, (arenaPnt1[0], arenaPnt1[1]), (arenaPnt2[0], arenaPnt2[1]), (0, 0, 255), 1)
	#cv2.rectangle(im, (badSquare1[0], badSquare1[1]), (badSquare2[0], badSquare2[1]), (0, 0, 255), 10)

	#cv2.imshow("Map", im)

	#cv2.waitKey(0)
	#cv2.destroyAllWindows()

if __name__ == '__main__': main()
