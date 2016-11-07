#!/usr/bin/env python

"""
    base_arm_nav.py - Version 1.1 2016-07-20
    
    Based on the R. Patrick Goebel's arm_tracker.py demo code
    
    Move the arm to point to a target on the /poi topic
    
    Created for the CCAM project

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
import moveit_commander
from moveit_commander import MoveGroupCommander
import tf
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped, PointStamped, PoseArray
from sensor_msgs.msg import JointState
from scipy.spatial.distance import euclidean 
import shlex, subprocess
import os
from math import sqrt, acos, radians, pi
from move_base_msgs.msg._MoveBaseActionFeedback import MoveBaseActionFeedback
import sys
from tf.TransformerROS import transformPoint


class BaseArmNavigation(object):
    def __init__(self, node_name):
        self.node_name = node_name
        
        rospy.init_node(node_name)
        
        rospy.loginfo("Starting node " + str(node_name))
        
        rospy.on_shutdown(self.shutdown)
        
        #Initialize necessary variables
        self.arm_group_name = rospy.get_param("~arm_group_name", "ur5_arm")
        self.stow_group_state = rospy.get_param("~stow_group_state", "stow")
        self.work_group_state = rospy.get_param("work_group_state", "work")
        self.reference_frame = rospy.get_param("~reference_frame", "map")
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "odom")
        self.initial_location_topic = rospy.get_param("~initial_location_topic", "/initial_locations")
        self.poi_topic = rospy.get_param("~poi_topic", "/poi")
        self.error_topic = rospy.get_param("~error_topic", "/error")
        self.allow_replanning = rospy.get_param("~allow_replanning", True )
        self.position_tolerance = rospy.get_param("~position_tolerance", .0001)
        self.orientation_tolerance = rospy.get_param("~orientation_tolerance", .0001)
        self.desired_orientation = rospy.get_param("~desired_orientation", Quaternion(0, 0, 0, 1))
        self.cartestian_path = rospy.get_param("~cartestian_path", False)
        self.cp_max_attempts = rospy.get_param("~cp_max_attempts", 100)
        self.workface_offset = rospy.get_param("~workface_offset", 1.0)
        
        # Initialize a number of global variables
        self.poi = None
        self.error = None
        
        # Arm Navigation Initialization
        # Initialize the move group for the right arm
        self.arm = MoveGroupCommander(self.arm_group_name)
        
        # Set the arm reference frame accordingly
        self.arm.set_pose_reference_frame(self.reference_frame)
                        
        # Allow replanning to increase the chances of a solution
        self.arm.allow_replanning(self.allow_replanning)
                
        # Set a position tolerance in meters
        self.arm.set_goal_position_tolerance(self.position_tolerance)
        
        # Set an orientation tolerance in radians
        self.arm.set_goal_orientation_tolerance(self.orientation_tolerance)
        
        # What is the end effector link?
        self.end_effector_link = self.arm.get_end_effector_link() 
        
        # Move Base Initialization
        # Publisher to manually control the robot
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # Initialize Tf
        # Create the transform listener
        self.listener = tf.TransformListener()
        # Queue Tf data
        rospy.sleep(3)
        rospy.loginfo("Listening to Tfs")
        
        # Subscribe to necessary topics
        # Subscribe to the initial locations topic and store them as self.initial_locations
        self.initial_locations = rospy.wait_for_message(self.initial_location_topic, PoseArray)
        
        # Subscribe to the poi topic
        rospy.wait_for_message(self.poi_topic, PointStamped)
        
        # Use queue_size=1 so outdated poi and error messages are not piled up
        self.poi_subscriber = rospy.Subscriber(self.poi_topic, PointStamped, self.update_poi, queue_size=1)
        self.error_subscriber = rospy.Subscriber(self.error_topic, PointStamped, self.update_error, queue_size=1)
        
        rospy.loginfo("Subscribed to necessary topics")
        rospy.loginfo("Ready to perform task")
        
        # Perform desired task
        self.perform_task()
         
    def perform_task(self):
        rospy.loginfo("Performing specified task")
        return
    
    def update_poi(self, poi):
        self.poi = poi
    
    def update_error(self, error):
        self.error = error
        
    def convert_point_to_arm_goal(self, point, initial_location):
        #Create Pose Msgs
        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        pose.header.stamp = rospy.Time.now()     
        pose.pose.position.x = initial_location.pose.position.x
        pose.pose.position.y = point.point.y
        pose.pose.position.z = point.point.z
        pose.pose.orientation = initial_location.pose.orientation
        
        return pose
    
    def convert_pose_to_move_base_goal(self, goal):
        # Initialize the move_base_goal
        move_base_goal = MoveBaseGoal()
            
        # Use the map frame to define goal poses
        move_base_goal.target_pose.header.frame_id = self.reference_frame
            
        # Set the time stamp to "now"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set the pose
        move_base_goal.target_pose.pose = Pose(Point(goal.pose.position.x - self.workface_offset, goal.pose.position.y, 0.0), goal.pose.orientation)
        
        return move_base_goal
        
    def plan_cartesian_path(self, end_pose):
        #Initialize necessary variables
        fraction = 0.0
        attempts = 0
        start_pose = self.arm.get_current_pose(self.end_effector_link)

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()
     
        # Plan the Cartesian path connecting the start point and goal
        while fraction < 1.0 and attempts < self.cp_max_attempts:
            (plan, fraction) = self.arm.compute_cartesian_path([start_pose, end_pose],   # waypoint poses
                                                                0.01,        # eef_step
                                                                0.0,         # jump_threshold
                                                                True)        # avoid_collisions
            # Increment the number of attempts 
            attempts += 1
                         
        # If we have a complete plan, return that plan
        if fraction == 1.0:
            return plan
        else:
            return None
        
    def plan_regular_path(self, end_pose):
        # Set the start state to the current state
        self.arm.set_start_state_to_current_state()

        # Set the goal pose of the end effector to the stored pose
        self.arm.set_pose_target(end_pose, self.end_effector_link)

        # Plan the trajectory to the goal
        plan = self.arm.plan()
        
        return plan
    
    def arm_nav_predicition(self, goal):
        # Initialize the outcome to False
        outcome = False
        
        if self.cartestian_path:
            plan = self.plan_cartesian_path(goal)
        else:
            plan = self.plan_regular_path(goal)
        
        if plan is not None:
            # Execute the planned trajectory
            self.arm.execute(plan)
            outcome = True
            
        return outcome
        
    def arm_nav_predicition_with_move_base(self, goal):
        # Initialize the outcome to False
        outcome = False
        
        if self.cartestian_path:
            plan = self.plan_cartesian_path(goal)
        else:
            plan = self.plan_regular_path(goal)
        
        if plan is not None:
            # Execute the planned trajectory
            self.arm.execute(plan)
            outcome = True
        else:
            move_base_goal = self.convert_pose_to_move_base_goal(goal)
            base_move_outcome = self.base_move(move_base_goal)
            if base_move_outcome:
                if self.cartestian_path:
                    plan = self.plan_cartesian_path(goal)
                else:
                    plan = self.plan_regular_path(goal)
        
                if plan is not None:
                    # Execute the planned trajectory
                    self.arm.execute(plan)
                    outcome = True
                    
        return outcome
    
    def arm_nav_correction(self, goal):
        # Initialize the outcome to False
        outcome = False
        
        if self.cartestian_path:
            plan = self.plan_cartesian_path(goal)
        else:
            plan = self.plan_regular_path(goal)
        
        if plan is not None:
            # Execute the planned trajectory
            self.arm.execute(plan)
            outcome = True
        
        return outcome
        
    def arm_nav_correction_with_move_base(self, goal):
        # Initialize the outcome to False
        outcome = False
        
        if self.cartestian_path:
            plan = self.plan_cartesian_path(goal)
        else:
            plan = self.plan_regular_path(goal)
        
        if plan is not None:
            # Execute the planned trajectory
            self.arm.execute(plan)
            outcome = True
        else:
            move_base_goal = self.convert_pose_to_move_base_goal(goal)
            base_move_outcome = self.base_move(move_base_goal)
            if base_move_outcome:
                if self.cartestian_path:
                    plan = self.plan_cartesian_path(goal)
                else:
                    plan = self.plan_regular_path(goal)
        
                if plan is not None:
                    # Execute the planned trajectory
                    self.arm.execute(plan)
                    outcome = True
                    
        return outcome
    
    def base_move(self, goal):
        # Initialize the outcome to False
        base_move_outcome = False
        
        # Get the current position of the system's base
        if self.listener.canTransform(self.reference_frame, self.robot_base_frame, rospy.Time.now()):
            try:
                start_position = PoseStamped()
                start_position.header.frame_id = self.robot_base_frame
                start_position.header.stamp = rospy.Time.now()
                start_position.pose = Pose(Point(0.0,0.0,0.0), Quaternion(0.0,0.0,0.0,1))

                start_position = self.listener.transformPose(self.reference_frame, start_position)
            except:
                print "TF execption in base_arm_nav.base_move"
        else:
            return
        
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)
            
        # Allow 1 minute for the system to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
            
        # If the system doesn't get there in time, abort the goal and return
        # to the start position
        if not finished_within_time:
            # Cancel current goal
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            # Move back to start_position
            self.move_base.send_goal(start_position)
        else:
            # The system made it to the goal position
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                #Set the outcome to True
                base_move_outcome = True
            else:
                self.move_base.send_goal(start_position)
        return base_move_outcome
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot")
        # Stop any further target messages from being processed
        self.poi_subscriber.unregister()
        self.error_subscriber.unregister()
    
        # Stop any current arm movement
        self.arm.stop()
        
        # Cancel any active move_base goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        
        # Stop the robot from moving
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        
        # Move the arm to the stow position
        self.arm.set_named_target(self.stow_group_state)
        self.arm.go()
        
        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
def main(args):
    try:
        node_name = "BaseArmNavigation"
        BaseArmNavigation(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down BaseArmNavigation node."
        
if __name__ == "__main__":
    main(sys.argv)
        
