#!/usr/bin/env python

""" autonomous.py - Version 1.0 2016-10-12

    General framework based on Patrick Goebel's nav_test.py
    Initial version based on ccam-navigation by Chris Mobley
    Autonomous movement added by Jonathan Hodges

    This code moves a mobile base along a waypoint based search routine
    while looking for an object. Once an object is detected, the search
    routine is cancelled and the mobile base moves toward the object
    instead.

    Input Files:
        mbzirc_c2_auto/params/pre-defined-path.txt - Waypoints for search
                                                     routine
    Subscribers:
        /detection- array containing [angle,distance] to the median of
                    the detected object in local coordinates. Contains
                    [0,0] if no object is detected.
    Publishers:
        /cmd_vel - topic to manually move the robot
        /move_base/goal - goal sent to the move_base node in ROS
        
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

    def __init__(self):
        """
          A few key tasks are achieved in the initializer function:
              1. We load the pre-defined search routine
              2. We connect to the move_base server in ROS
              3. We start the ROS subscriber callback function registering
                 the object
              4. We initialize counters in the class to be shared by the
                 various callback routines

            These are the possible returns from move_base.get_state() function.
            Included for reference. 
            goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                           'SUCCEEDED', 'ABORTED', 'REJECTED',
                           'PREEMPTING', 'RECALLING', 'RECALLED',
                           'LOST']
        """
        # Establish ROS node
        rospy.init_node('autonomous', anonymous=True) # Name this node
        rospy.on_shutdown(self.shutdown) # Enable shutdown in rospy
        rospack = rospkg.RosPack() # Find rospackge locations

        # Initialize parameters
        self.rest_time = 0.1            # Minimum pause at each location
        self.stalled_threshold = 200    # Loops before stall

        # Set up the waypoint locations. Poses are defined in the map frame.
        self.locations = dict()
        self.wpname = dict()
        
        f = open(rospack.get_path(
            'mbzirc_c2_auto')+'/params/pre-defined-path.txt','r')
        line_counter = 0
        with f as openfileobject:
            first_line = f.readline()
            for line in openfileobject:
                nome = [x.strip() for x in line.split(',')]
                self.wpname[line_counter] = nome[0]
                x = Decimal(nome[1]); y = Decimal(nome[2]); z = Decimal(nome[3])
                X = Decimal(nome[4]); Y = Decimal(nome[5]); Z = Decimal(nome[6])
                self.locations[self.wpname[line_counter]] = Pose(Point(x,y,z), Quaternion(X,Y,Z,1))
                line_counter = line_counter+1
        self.wp = 0
        self.ct = 0
        self.ct4 = 0
        self.ct5 = 0
        self.state = 3

        # Set up ROS publishers and subscribers
        # Publisher to manually control the robot 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to object detection topic
        rospy.Subscriber("/detection", numpy_msg(Floats), self.callback, queue_size=1)
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        self.goal = MoveBaseGoal()
        rospy.sleep(self.rest_time)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def callback(self, bearing):
        # back_it_up - this subroutine moves the husky forward or
        # backward a distance by manually controlling the wheels
        def back_it_up(ve,dist_to_move):
            sleep_time = 0.1
            time_to_move = abs(dist_to_move/ve)
            twist = Twist()
            twist.linear.x = ve
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            twi_pub = rospy.Publisher("/joy_teleop/cmd_vel", Twist, queue_size=10)
            self.ct_move = 0
            while self.ct_move*sleep_time < time_to_move:
                twi_pub.publish(twist)
                self.ct_move = self.ct_move+1
                rospy.sleep(sleep_time)
            return None

        # rot_cmd - this subroutine rotates the husky 90 degrees by
        # controlling the wheels manually
        def rot_cmd(ve,dist_to_move):
            sleep_time = 0.1
            time_to_move = abs(dist_to_move/ve)
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = ve
            twi_pub = rospy.Publisher(
                "/joy_teleop/cmd_vel", Twist, queue_size=10)
            self.ct_move = 0
            while self.ct_move*sleep_time < time_to_move:
                twi_pub.publish(twist)
                self.ct_move = self.ct_move+1
                rospy.sleep(sleep_time)
            return None

        #if self.wp > -1:
        #    self.state = self.move_base.get_state()
        #else:
        #    self.goal.target_pose.pose = self.locations[self.wpname[self.wp+1]]
        #    self.goal.target_pose.header.frame_id = 'odom'
        #    self.goal.target_pose.header.stamp = rospy.Time.now()
        #    self.move_base.send_goal(self.goal)
        #    self.wp = self.wp+1
        #    self.ct = 0
        #    rospy.sleep(5)
        #    self.state = self.move_base.get_state()
        if bearing.data[0] == 0:
            try:
                self.state = self.move_base.get_state()
            except:
                self.state = self.state
            rospy.logdebug("%f",self.ct)
            self.ct5 = self.ct5+1
            rospy.logdebug("I see no object!")
            if self.state == 3:
                self.wp = self.wp+1
                location = self.wpname[self.wp]
                self.goal.target_pose.pose = self.locations[location]
                self.goal.target_pose.header.frame_id = 'odom'
                rospy.loginfo("Going to: " + str(location))
                self.move_base.send_goal(self.goal)
                self.ct = 0
                self.ct4 = 0
                rospy.sleep(2)
            else:
                self.ct = self.ct+1
                if self.ct > self.stalled_threshold:
                    rospy.loginfo("We are stuck!")
                    rospy.loginfo("Applying catch routine.")
                    self.move_base.cancel_goal()
                    rospy.sleep(1)
                    back_it_up(-0.25,0.5)
                    rospy.sleep(0.1)
                    rot_cmd(0.25,0.25)
                    rospy.loginfo("Resending goal.")
                    self.move_base.send_goal(self.goal)
                    rospy.sleep(1)
                    self.ct_move = 0
                    self.ct = 0
            if self.ct5 > 5:
                self.ct4 = 0
        else:
            if self.ct4 > 10:
                rospy.logdebug("Object identified.")
                if bearing.data[1] > 10:
                    bear = 10
                else:
                    bear = bearing.data[1]
                if bear < 3:
                    bear = 0
                x = bear*np.cos(bearing.data[0])-1
                y = bear*np.sin(bearing.data[0])-1
                self.goal.target_pose.header.frame_id = 'base_link'
                self.goal.target_pose.pose = Pose(Point(x,y,0), Quaternion(0,0,0,1))
                rospy.loginfo("Going to: (%f,%f)",bearing.data[0],bearing.data[1])
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.send_goal(self.goal)
                try:
                    self.state = self.move_base.get_state()
                except:
                    self.state = self.state
                self.ct4 = self.ct4+1
                if bear < 3:
                    rospy.set_param('smach_state','atBoard')
                    rospy.signal_shutdown('We are close enough to the object!')
                rospy.sleep(5)
                rospy.loginfo("State:" + str(self.state))
            else:
                self.ct4 = self.ct4+1
                self.ct5 = 0

if __name__ == '__main__':
    try:
        mbzirc_c2_auto()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("mbzirc_c2_auto finished.")

