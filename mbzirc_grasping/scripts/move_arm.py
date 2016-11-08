#!/usr/bin/env python

"""
    grasp.py - Version 0.1 2015-11-05
    Use inverse kinemtatics to move the end effector to grab the wrench.
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
import sys
import moveit_commander
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
import roslib
roslib.load_manifest("rosparam")
import rosparam

def callback(data):
	rospy.loginfo("Moving arm to desired position")

	# Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        arm = moveit_commander.MoveGroupCommander("ur5_arm")

        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()

        # Initialize Necessary Variables
        reference_frame = rospy.get_param("~reference_frame", "/base_link")

        # Set the ur5_arm reference frame accordingly
        arm.set_pose_reference_frame(reference_frame)

	# Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)
        
        #Set the target pose from the input
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = data.position[0]
        target_pose.pose.position.y = data.position[1]
        target_pose.pose.position.z = data.position[2]
        target_pose.pose.orientation.x = data.position[3]
        target_pose.pose.orientation.y = data.position[4]
        target_pose.pose.orientation.z = data.position[5]
        target_pose.pose.orientation.w = data.position[6]

	# Set the start state to the current state
        arm.set_start_state_to_current_state()

        # Set the goal pose of the end effector to the stored pose
        arm.set_pose_target(target_pose, end_effector_link)

        # Plan the trajectory to the goal
        traj = arm.plan()
        
        if traj is not None:
            # Execute the planned trajectory
            arm.execute(traj)
            
            # Pause for a second
            rospy.sleep(5.0)
                
            rospy.loginfo("Successfully moved")
                
        else:
            rospy.loginfo("Unable to reach")

def main():
	rospy.init_node("move_arm", anonymous=False)

	rospy.set_param('arm_prefix', 'ur5_arm_')
	rospy.set_param('reference_frame', '/base_link')

	rospy.Subscriber("/whatevertellsthearmwheretogo", JointState, callback)

	rospy.spin()

if __name__ == '__main__': main()
