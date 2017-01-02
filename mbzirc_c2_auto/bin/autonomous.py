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
import numpy as np
from decimal import *

class mbzirc_c2_auto():
    """MBZIRC Challenge 2 autonomous searching routine.

    This class moves a mobile base along a waypoint based search routine while
    looking for an object. Once an object is detected, the search routine is
    cancelled and the mobile base moves toward the object instead.

    Attributes:
        locations: pose of waypoints
        waypoint_name: name of waypoints
        rest_time: default sleep time
        stalled_threshold: threshold for stalled UGV
        current_waypoint: current waypoint
        stall_counter: counter for stalled UGV
        detect_counter: counter for LIDAR detection scans
        noise_counter: counter for LIDAR empty scans
        state: current state of move_base node
        twi_pub: Publisher to manually control the robot 
        move_base: move_base action server
        goal: move_base goal
        ct_move: manually movement counter

    Dependencies:
        mbzirc_c2_auto/params/pre-defined-path.txt: search routine waypoints

    Subscribers:
        /detection: array containing [angle,distance] to the median of the
            detected object in local coordinates. Contains [0,0] if no object
            is detected.

    Publishers:
        /joy_teleop/cmd_vel: topic to manually move the robot
        /move_base/goal: goal sent to the move_base node in ROS
            
    """

    def __init__(self):
        """This initializes the various attributes in the class

        A few key tasks are achieved in the initializer function:
            1. We load the pre-defined search routine
            2. We connect to the move_base server in ROS
            3. We start the ROS subscriber callback function registering
               the object
            4. We initialize counters in the class to be shared by the various
               callback routines

        These are the possible returns from move_base.get_state() function.
        Included for reference:
        goal_states: ['PENDING', 'ACTIVE', 'PREEMPTED','SUCCEEDED', 'ABORTED',
                      'REJECTED','PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

        """
        # Establish ROS node
        rospy.init_node('autonomous', anonymous=True) # Name this node
        rospy.on_shutdown(self.shutdown) # Enable shutdown in rospy
        rospack = rospkg.RosPack() # Find rospackge locations

        # Set up the waypoint locations. Poses are defined in the map frame.
        self.locations = dict()
        self.waypoint_name = dict()
        
        f = open(rospack.get_path(
            'mbzirc_c2_auto')+'/params/pre-defined-path.txt','r')
        line_counter = 0
        with f as openfileobject:
            first_line = f.readline()
            for line in openfileobject:
                nome = [x.strip() for x in line.split(',')]
                self.waypoint_name[line_counter] = nome[0]
                x=Decimal(nome[1]); y=Decimal(nome[2]); z=Decimal(nome[3])
                X=Decimal(nome[4]); Y=Decimal(nome[5]); Z=Decimal(nome[6])
                self.locations[self.waypoint_name[line_counter]] = Pose(
                    Point(x,y,z), Quaternion(X,Y,Z,1))
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
        self.twi_pub = rospy.Publisher("/joy_teleop/cmd_vel", Twist,
            queue_size=5)
        # Subscribe to object detection topic
        rospy.Subscriber("/detection", numpy_msg(Floats), self.callback, 
            queue_size=1)
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", 
            MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        self.goal = MoveBaseGoal()

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        rospy.sleep(self.rest_time)

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

    def callback(self, bearing):
        """This callback occurs whenever the object detection script publishes
        to the /detection topic. If the array is [0,0], no object was detected
        and we move the UGV along the pre-defined search routine. If the array
        is not [0,0], we move the UGV toward the object.
        """
        def back_it_up(ve,dist_to_move):
            """This subroutine manually moves the husky forward or backward
            a fixed amount at a fixed velocity by bypassing the move_base
            package and sending a signal directly to the wheels.
                
            This subroutine is likely to be replaced by:
                file:
                    robot_mv_cmds
                    subroutine: move_UGV_vel(lv,av,dist_to_move)
            """ 
            sleep_time = 0.1
            time_to_move = abs(dist_to_move/ve)
            twist = Twist()
            twist.linear.x = ve
            self.ct_move = 0
            while self.ct_move*sleep_time < time_to_move:
                self.twi_pub.publish(twist)
                self.ct_move = self.ct_move+1
                rospy.sleep(sleep_time)
            return None

        def rot_cmd(ve,dist_to_move):
            """This subroutine manually rotates the husky along the z-axis a
            fixed amount at a fixed velocity by bypassing the move_base package
            and sending a signal directly to the wheels.

            This subroutine is likely to be replaced by:
                file:
                    robot_mv_cmds
                    subroutine: move_UGV_vel(lv,av,dist_to_move)
            """
            sleep_time = 0.1
            time_to_move = abs(dist_to_move/ve)
            twist = Twist()
            twist.angular.z = ve
            self.ct_move = 0
            while self.ct_move*sleep_time < time_to_move:
                self.twi_pub.publish(twist)
                self.ct_move = self.ct_move+1
                rospy.sleep(sleep_time)
            return None

        if bearing.data[0] == 0:
            """If /detection topic is publishing [0,0] then we do not see an
            object. Move along the predefined path.
            """
            try:
                self.state = self.move_base.get_state()
            except:
                pass
            self.noise_counter = self.noise_counter+1
            rospy.logdebug("I see no object!")
            if self.state == 3:
                # If move_base is registering 'SUCCEEDED' move to next waypoint
                self.current_waypoint = self.current_waypoint+1
                location = self.waypoint_name[self.current_waypoint]
                self.goal.target_pose.pose = self.locations[location]
                self.goal.target_pose.header.frame_id = 'odom'
                rospy.loginfo("Going to: " + str(location))
                self.move_base.send_goal(self.goal)
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
            if self.noise_counter > 5:
                # Check if we have gotten more blank scans than allowed. Reset
                # the detection counter.
                self.detect_counter = 0
        else:
            """If /detection is not publishing [0,0], we found an object! To
            remove noisy signals, we make sure we find 10 valid scans.
            """
            if self.detect_counter > 10:
                # Check number of valid scans to identify the object.
                rospy.logdebug("Object identified.")

                # move_base doesn't like targets that are too far away. If the
                # range is too far away, fix the range to 10m.
                if bearing.data[1] > 10:
                    bear = 10
                else:
                    bear = bearing.data[1]
                # If the UGV is within 3m of the target, set the goal to 0
                if bear < 3:
                    bear = 0
                # Create target location in the local reference frame from the
                # /detection angle and range
                x = bear*np.cos(bearing.data[0])-1
                y = bear*np.sin(bearing.data[0])-1
                self.goal.target_pose.header.frame_id = 'base_link'
                self.goal.target_pose.pose = Pose(Point(x,y,0),
                    Quaternion(0,0,0,1))
                rospy.loginfo("Going to: (%f,%f)",
                    bearing.data[0],bearing.data[1])
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
                try:
                    self.state = self.move_base.get_state()
                except:
                    pass
                self.detect_counter = self.detect_counter+1
                if bear < 3:
                    rospy.set_param('smach_state','atBoard')
                    rospy.signal_shutdown('We are close enough to the object!')
                rospy.sleep(2)
                rospy.loginfo("State:" + str(self.state))
            else:
                self.detect_counter = self.detect_counter+1
                self.noise_counter = 0

if __name__ == '__main__':
    try:
        mbzirc_c2_auto()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("mbzirc_c2_auto finished.")

