#!/usr/bin/env python

""" mbzirc_c2_auto.py - Version 1.0 2016-10-12

    General framework based on Patrick Goebel's nav_test.py
    Initial version based on ccam-navigation by Chris Mobley

    Define waypoint destinations for a robot to move autonomously within
    a map framework.

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
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
from decimal import *
import tf

class mbzirc_c2_auto():

    def __init__(self):
	rospy.init_node('autonomous', anonymous=True) # Give the node a name
        rospy.on_shutdown(self.shutdown) # Set rospy to execute a shutdown function when exiting
        self.rest_time = rospy.get_param("~rest_time", 0.1) # Minimum pause at each location
        self.stalled_threshold = rospy.get_param("~stalled_threshold", 100) # Loops before stall

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
        self.locations = dict()
        self.wpname = dict()
        
        rospack = rospkg.RosPack()
        f = open(rospack.get_path('mbzirc_c2_auto')+'/params/pre-defined-path.txt','r')
        ct2 = 0
        with f as openfileobject:
            first_line = f.readline()
            for line in openfileobject:
                nome = [x.strip() for x in line.split(',')]
                self.wpname[ct2] = nome[0]
                x = Decimal(nome[1]); y = Decimal(nome[2]); z = Decimal(nome[3])
                X = Decimal(nome[4]); Y = Decimal(nome[5]); Z = Decimal(nome[6])
                self.locations[self.wpname[ct2]] = Pose(Point(x,y,z), Quaternion(X,Y,Z,1))
                ct2 = ct2+1
        self.wp = -1

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        self.goal = MoveBaseGoal()
        rospy.sleep(self.rest_time)
        rospy.Subscriber("/detection", numpy_msg(Floats), self.callback, queue_size=1)
        rospy.Subscriber("/move_base/feedback", MoveBaseFeedback, self.callback_feedback, queue_size=1)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def callback_feedback(self, data):
        self.feedback = data
        
    def callback(self, bearing):
        if self.wp > -1:
            state = self.move_base.get_state()
        else:
            self.goal.target_pose.pose = self.locations[self.wpname[self.wp+1]]
            self.goal.target_pose.header.frame_id = 'odom'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            self.wp = self.wp+1
            self.ct = 0
            state = self.move_base.get_state()
        if bearing.data[0] == 0:
            rospy.loginfo("%f",self.ct)
            if state == 3:
                self.wp = self.wp+1
                location = self.wpname[self.wp]
                self.goal.target_pose.pose = self.locations[location]
                self.goal.target_pose.header.frame_id = 'odom'
                rospy.loginfo("Going to: " + str(location))
                self.move_base.send_goal(self.goal)
                self.ct = 0
            else:
                self.ct = self.ct+1
                if self.ct > self.stalled_threshold:
                    rospy.loginfo("Potentially stuck. Resending goal.")
                    location = self.wpname[self.wp]
                    self.goal.target_pose.pose = self.locations[location]
                    self.goal.target_pose.header.frame_id = 'odom'
                    self.move_base.send_goal(self.goal)
                    self.ct = 0
        else:
            x0 = self.feedback.feedback.base_position.pose.position.x
            y0 = self.feedback.feedback.base_position.pose.position.y
            z0 = self.feedback.feedback.base_position.pose.position.z
            X0 = self.feedback.feedback.base_position.pose.orientation.x
            Y0 = self.feedback.feedback.base_position.pose.orientation.y
            Z0 = self.feedback.feedback.base_position.pose.orientation.z
            W0 = self.feedback.feedback.base_position.pose.orientation.w

            if bearing.data[1] > 10:
                bear = 10
            else:
                bear = bearing.data[1]
            if bear < 3:
                bear = 0
            x = bear*np.cos(bearing.data[0]-Z0)-1+x0-1
            y = -bear*np.sin(bearing.data[0]-Z0)-1+y0
            print bearing.data[0], Z0
            self.goal.target_pose.header.frame_id = 'odom'
            self.goal.target_pose.pose = Pose(Point(x,y,0), Quaternion(0,0,0,1))
            rospy.loginfo("Going to: (%f,%f)",x,y)
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            state = self.move_base.get_state()
            if bear < 3:
                rospy.signal_shutdown('We are close enough to the object!')
            rospy.sleep(5)
            rospy.loginfo("State:" + str(state))

if __name__ == '__main__':
    try:
        mbzirc_c2_auto()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("mbzirc_c2_auto finished.")

