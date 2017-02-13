#!/usr/bin/env python

""" ccam_navigation.py - Version 1.1 2016-01-01

    Based on Patrick Goebel's nav_test.py

    Command a robot to move autonomously to a specified named goal locations
    within a defined in the map frame.

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
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class CCAMNavigation():
    def __init__(self):
	# Give the node a name
        rospy.init_node('ccam_navigation', anonymous=True)

	# Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

	# The minimum time the robot should pause at each location
        self.rest_time = rospy.get_param("~rest_time", 5)

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        locations = dict()

        locations['start'] = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        locations['finish'] = Pose(Point(4.0,-4.075,0.0), Quaternion(0,0,1.05,1))
        locations['sealant_station_1'] = Pose(Point(2.625,2.0646,0.0), Quaternion(0,0,1.05, 0.0))
        locations['sealant_station_2'] = Pose(Point(2.625,5.8694,0.0), Quaternion(0,0,1.05, 0.0))
        locations['drill_station_1'] = Pose(Point(7.6912,6.875,0.0), Quaternion(0,0,1.05,1))
        locations['drill_station_2'] = Pose(Point(13.1912,6.875,0.0), Quaternion(0,0,1.05,1))
        locations['charging_station'] = Pose(Point(9.0,-4.075,0.0), Quaternion(0,0,1.05,1))
        locations['tool_change_station'] = Pose(Point(14.0,-4.075,0.0), Quaternion(0,0,1.05,1))

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()

        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        rospy.loginfo("Starting navigation test")

        # Begin the main loop and navigation to user specified locations
        while not rospy.is_shutdown():

            location = raw_input('Enter desired location (ie. start, drill_station_1, etc.): ')

            if location == "start" or location == "finish" or location == "sealant_station_1" or location == "sealant_station_2" or location == "drill_station_1" or location == "drill_station_2" or location == "charging_station" or location == "tool_change_station":

                # Set up the goal location
                self.goal = MoveBaseGoal()
                self.goal.target_pose.pose = locations[location]
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()

                # Let the user know where the robot is going next
                rospy.loginfo("Going to: " + str(location))

                # Start the robot toward user specified location
                self.move_base.send_goal(self.goal)

                # Allow 5 minutes to get there
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

                # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    rospy.loginfo("Timed out achieving goal")
                else:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Reached " + str(location))
                        rospy.loginfo("State:" + str(state))
                    else:
                    	rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

                rospy.sleep(self.rest_time)
            else:
                print "Unknown location selected. Please select a know area."


    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        CCAMNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("CCAM navigation finished.")
