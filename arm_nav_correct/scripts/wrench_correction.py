#!/usr/bin/env python

"""
    wrench_correcton.py - Version 0.1 2016-11-03

    Luan Cong Doan _ CMS Lab
    luandoan@vt.edu

    Use inverse kinemtatics to move the end effector from prediction point to the center point of wrench

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
from moveit_msgs.msg import RobotState
from control_msgs.msg import FollowJointTrajectoryActionResult
from actionlib_msgs.msg import *
import roslib
roslib.load_manifest("rosparam")
import rosparam
import locations

class WrenchCorrection()
    def __init__(self):
	rospy.init_node('wrench_correction', anonymous=False)
	rospy.on_shutdown(self.cleanup) 	# shutdown function when executing done
	self.flag = 0
	self.ct = 0

	rospy.Subscriber("move_group/status", GoalStatusArray, self.callback, queue_size=1)
	
	rospy.loginfo("Wrench Correction step")

	# Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander("ur5_arm")

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Initialize Necessary Variables
        self.reference_frame = rospy.get_param("~reference_frame", "/base_link")

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(self.reference_frame)

	# Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)
        
        #Set the target pose from the input
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = reference_frame
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.pose.position.x = locations.point.x
        self.target_pose.pose.position.y = locations.point.y
        self.target_pose.pose.position.z = locations.point.z

	# Set the start state to the current state
	try:
	    cjs = rospy.get_param('current_joint_state')
	except:
	    cjs = [0, 0, 0, 0, 0, 0]

	jt = RobotState()	# Get current state from Robot State
        jt.joint_state.header.frame_id = '/base_link'
        jt.joint_state.name = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel', 'ur5_arm_shoulder_pan_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_elbow_joint', 'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint', 'left_tip_hinge', 'right_tip_hinge']
        jt.joint_state.position = [0,0,0,0,cjs[0],cjs[1],cjs[2],cjs[3],cjs[4],cjs[5],0,0]

	self.arm.set_start_state(jt)

        # Set the goal pose of the end effector to the stored pose
        self.arm.set_pose_target(self.target_pose, self.end_effector_link)

        # Plan the trajectory to the goal
        traj = arm.plan()
        
        if traj is not None:
            # Execute the planned trajectory
            arm.execute(traj)
            rospy.loginfo("Moving to wrench location")
            # Pause for a second
            rospy.sleep(3.0)
	    # Moving toward the wrench
            rospy.loginfo("Approaching wrench")
            arm.shift_pose_target(1, 0.12, end_effector_link)
            arm.go()
            rospy.loginfo("Moving forward 12cm")
            rospy.sleep(3.0)
            rospy.loginfo("Approaching successfully")    
            rospy.loginfo("Successfully moved")
                
        else:
            rospy.loginfo("Unable to reach")

        
        #plan = arm.plan()
        #if plan is not None:
        #    arm.execute(plan)
        #    rospy.loginfo("Moving forward")
        #    rospy.sleep(5.0)
        #    rospy.loginfo("Reached the right wrench")
        #else:
        #    rospy.loginfo("Unable to reach the wrench")	
def callback(self, data):
    if self.flag ==1:
	self.status = data.status_list[0].status
	    if self.status ==3:
		rospy.sleep(5)
		rospy.set_param('move_arm_status', 'success')
		self.flag = 2
	    else:
		if self.ct > 10:
		    rospy.set_param('move_arm_status', 'failure')
		    self.flag = 2
	 	else:
		    self.ct = self.ct + 1
		    rospy.sleep(1.0)
    if self.flag ==2:
	self.cleanup()
	rospy.signal_shutdown('Ending node.')
def cleanup(self):
    rospy.loginfo("Stopping the robot")
    self.arm.stop()
    # Shut down MoveIt
    rospy.loginfo("Shutting down MoveIt")
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == '__main__':
    WrenchCorrection

 
