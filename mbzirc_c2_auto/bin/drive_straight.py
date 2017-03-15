#!/usr/bin/env python

"""autonomous.py - Version 1.0 2016-10-12
General framework based on Patrick Goebel's nav_test.py
Initial version based on ccam-navigation by Chris Mobley
Autonomous movement added by Jonathan Hodges
This code moves a mobile base along a waypoint based search routine while
looking for an object. Once an object is detected, the search routine is
cancelled and the mobile base moves toward the object instead.
Dependencies:
    mbzirc_c2_auto/params/pre-defined-path.txt - Waypoints for search routine
Subscribers:
    /detection: array containing [angle,distance] to the median of
        the detected object in local coordinates. Contains [0,0] if no object
        is detected.
Publishers:
    /cmd_vel: topic to manually move the robot
    /move_base/goal: goal sent to the move_base node in ROS
This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html
"""

import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int8
import numpy as np
from decimal import *
import tf

class drive_straight():

    def __init__(self):
        # Establish ROS node
        rospy.init_node('drive_straight', anonymous=True) # Name this node
        rospy.on_shutdown(self.shutdown) # Enable shutdown in rospy
        rospack = rospkg.RosPack() # Find rospackge locations

        # Set up the waypoint locations. Poses are defined in the map frame.
        self.locations = dict()
        self.waypoint_name = dict()

        f = open(rospack.get_path(
            'mbzirc_c2_auto')+'/params/drive_straight.txt','r')
        line_counter = 0
        with f as openfileobject:
            first_line = f.readline()
            for line in openfileobject:
                nome = [x.strip() for x in line.split(',')]
                self.waypoint_name[line_counter] = nome[0]
                x=Decimal(nome[1]); y=Decimal(nome[2]); z=Decimal(nome[3])
                X=float(nome[4]); Y=float(nome[5]); Z=float(nome[6])
                q = tf.transformations.quaternion_from_euler(X, Y, Z)
                self.locations[self.waypoint_name[line_counter]] = Pose(
                    Point(x,y,z), Quaternion(q[0],q[1],q[2],q[3]))
                line_counter = line_counter+1

        # Initialize parameters
        self.rest_time = 0.1            # Minimum pause at each location
        self.stalled_threshold = 5000   # Loops before stall
        self.current_waypoint = 0       # Current waypoint
        self.stall_counter = 0          # Stall counter
        self.detect_counter = 0         # Detection counter
        self.noise_counter = 0          # Noise counter
        self.state = 3                  # move_base current state

        # Set up ROS publishers and subscribers
        # Publisher to manually control the robot
        self.twi_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base",
            MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        self.goal = MoveBaseGoal()

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Driving forward.")
        rospy.Subscriber("/prepare_to_die", Int8, self.cb_inigo, queue_size=1)

        rospy.sleep(self.rest_time)
        # If move_base is registering 'SUCCEEDED' move to next waypoint
        self.current_waypoint = self.current_waypoint+1
        location = self.waypoint_name[self.current_waypoint]
        self.goal.target_pose.pose = self.locations[location]
        self.goal.target_pose.header.frame_id = 'odom'
        rospy.loginfo("Going to: " + str(location))
        self.move_base.send_goal(self.goal)
        print "Going to: "
        print self.goal
        self.stall_counter = 0
        self.detect_counter = 0
        rospy.sleep(self.rest_time*10)

        while True:
            try:
                self.state = self.move_base.get_state()
            except:
                pass
            self.noise_counter = self.noise_counter+1
            rospy.logdebug("I see no object!")
            if self.state == 3:
                # If move_base is registering 'SUCCEEDED' move to next waypoint
                self.current_waypoint = self.current_waypoint+1
                if self.current_waypoint == line_counter:
                    rospy.signal_shutdown('We have arrived!')
                    rospy.sleep(1)
                location = self.waypoint_name[self.current_waypoint]
                self.goal.target_pose.pose = self.locations[location]
                self.goal.target_pose.header.frame_id = 'odom'
                rospy.loginfo("Going to: " + str(location))
                self.move_base.send_goal(self.goal)
                print "Going to: "
                print self.goal
                self.stall_counter = 0
                self.detect_counter = 0
                rospy.sleep(2)
            else:
                # If not registering 'SUCCEEDED', increment stall_counter
                self.stall_counter = self.stall_counter + 1
                if self.stall_counter > self.stalled_threshold:
                    # If stall_counter is too high, enter stuck routine.
                    # Ideally this will be replaced with a catch routine in
                    # husky_navigation package.
                    rospy.loginfo("We are stuck!")
                    rospy.loginfo("Applying catch routine.")
                    self.move_base.cancel_goal()
                    rospy.sleep(0.1)
                    back_it_up(-0.25,0.5)
                    rospy.sleep(0.1)
                    rot_cmd(0.25,0.25)
                    rospy.loginfo("Resending goal.")
                    self.move_base.send_goal(self.goal)
                    rospy.sleep(1)
                    self.ct_move = 0
                    self.stall_counter = 0
                rospy.sleep(0.1)
            if self.noise_counter > 5:
                # Check if we have gotten more blank scans than allowed. Reset
                # the detection counter.
                self.detect_counter = 0

    def shutdown(self):
        """This subroutine runs when the autonomous node shutdown. It is
        important to cancel any move_base goals prior to ending the node so
        the robot does not keep moving after this node dies.
        """
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(0.1)
        self.twi_pub.publish(Twist())
        rospy.sleep(0.1)

    def cb_inigo(self, data):
        rospy.loginfo("My name is Inigo Montoya, you killed my father...")
        rospy.loginfo("Prepare to die!")
        rospy.signal_shutdown('Time to operate in manual!')



if __name__ == '__main__':
    drive_straight()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("drive_straight finished.")
