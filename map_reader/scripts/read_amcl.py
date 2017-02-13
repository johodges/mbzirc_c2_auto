#!/usr/bin/env python

import rospy
import yaml
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

def callback(data):
	currentX = data.pose.pose.position.x
	currentY = data.pose.pose.position.y
	currentR = data.pose.pose.orientation.z * math.pi

	rospy.set_param("/currentRobotX", currentX)
	rospy.set_param("/currentRobotY", currentY)
	rospy.set_param("/currentRobotR", currentR)

def main():
	rospy.init_node('map_reader', anonymous=True)
	with open("/home/tom/catkin_ws/src/mbzirc_c2_auto/map_reader/mapCoords.yaml", 'r') as f:
    		doc = yaml.load(f)
	f.close()
	
	arenaPnt1 = doc["arenaTopLeft"]
	arenaPnt2 = doc["arenaBotRight"]

	rospy.set_param("/arenaPnt1", arenaPnt1)
	rospy.set_param("/arenaPnt2", arenaPnt2)

	badSquare1 = doc["deadZoneTopLeft"]
	badSquare2 = doc["deadZoneBotRight"]

	rospy.set_param("/deadZone1", badSquare1)
	rospy.set_param("/deadZone2", badSquare2)

	rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback, queue_size=1)
	rospy.spin()

if __name__ == '__main__': main()
