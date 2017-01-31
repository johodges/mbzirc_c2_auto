#!/usr/bin/env python

"""
	Run this to change the ee_position to off of the valve
"""

import rospy

def main():
	rospy.init_node('off_valve', anonymous=True)

	ee_pos = rospy.get_param('ee_position')

	ee_pos[0] = ee_pos[0] - 0.1

	rospy.set_param('ee_position', ee_pos)

if __name__ == '__main__': main()
