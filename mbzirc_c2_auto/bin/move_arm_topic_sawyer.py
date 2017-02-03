#!/usr/bin/env python

"""
    move_arm_topic.py - Version 0.1 2015-11-05
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
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from moveit_msgs.msg import RobotState
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionResult
from actionlib_msgs.msg import *
import roslib
roslib.load_manifest("rosparam")
import rosparam
import tf

class move_arm_topic():
    def __init__(self):
        # Name this node, it must be unique
        rospy.init_node('move_arm', anonymous=False)
        rospy.on_shutdown(self.cleanup) # Set rospy to execute a shutdown function when exiting
        self.ready_for_goal = 0
        self.ct = 0

        # Set up ROS subscriber callback routines
        rospy.Subscriber("/move_group/status", GoalStatusArray, self.cb_stat, queue_size=1)
        rospy.Subscriber("/move_arm/goal",Twist, self.cb_goal, queue_size=1)
        #rospy.Subscriber("/move_arm/joint_states",JointState, self.cb_joint, queue_size=1)

        # Set up ROS publishers

        self.joint_pub = rospy.Publisher("/move_arm/joint_states",JointState, queue_size=1)

        rospy.set_param('arm_prefix', 'right_arm')
        rospy.set_param('reference_frame', '/base')
        rospy.loginfo("Moving arm to desired position")
	rospy.sleep(1)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        # Set up collision object for sawyer base
        """
        co = moveit_msgs.msg.CollisionObject()
        co.header.frame_id = 'base'
        co.id = 'sawyer_table'
        co.type = 'BOX'
        

        [0.31,0.31,0]
        [0.31,-0.31,0]
        [-0.31,0.31,0]
        [-0.31,-0.31,0]
        print collision_object
        """
        rospy.sleep(2)
        p = PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        p.pose.position.x = 0.
        p.pose.position.y = 0.
        p.pose.position.z = -0.36
        scene.add_box("table", p, (0.62, 0.62, 0.72))


        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander("right_arm")

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo(self.end_effector_link)

        # Initialize Necessary Variables
        reference_frame = rospy.get_param("~reference_frame", "/base")
        #reference_frame = "ee_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.0001)
        self.arm.set_goal_orientation_tolerance(0.001)

        # Set the target pose from the input
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = reference_frame

        # Get current joint position to use for planning
        rospy.loginfo("Initializing motion planning server.")
        rospy.loginfo("Listening for <Twist> published on </move_arm/goal>")

    #def cb_joint(self, data):
        #self.jt.joint_state.position = [0,0,0,0,cjs[0],cjs[1],cjs[2],cjs[3],cjs[4],cjs[5],0,0]
        #self.jt.joint_state = data

    def cb_goal(self, data):
        if self.ready_for_goal == 0:
            rospy.loginfo("Recieved new goal.")
            roll = data.angular.x
            pitch = data.angular.y
            yaw = data.angular.z
            quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
            self.target_pose.pose.position.x = data.linear.x
            self.target_pose.pose.position.y = data.linear.y
            self.target_pose.pose.position.z = data.linear.z
            self.target_pose.pose.orientation.x = quat[0]
            self.target_pose.pose.orientation.y = quat[1]
            self.target_pose.pose.orientation.z = quat[2]
            self.target_pose.pose.orientation.w = quat[3]
            self.target_pose.header.stamp = rospy.Time.now()

            # Set the start state to the current state
            self.arm.set_start_state_to_current_state()

            # Set the goal pose of the end effector to the stored pose
            self.arm.set_pose_target(self.target_pose, self.end_effector_link)

            # Plan the trajectory to the goal
            traj = self.arm.plan()
            traj_pts = len(traj.joint_trajectory.points)
            
            if traj is not None:
                # Execute the planned trajectory
                self.move_succeeded = self.arm.execute(traj)

                self.ready_for_goal = 1
        else:
            rospy.loginfo("Recieved new goal before finishing motion. Ignoring.")
    # callback_feedback is used to store the feedback topic into the class to be
    # referenced by the other callback routines.
    def cb_stat(self, data):
        if self.ready_for_goal == 1:
            self.status = data.status_list[0].status

            if self.move_succeeded:
                # rospy.sleep(5)
                rospy.loginfo('Successful Arm Move')
                rospy.set_param('move_arm_status','success')
                self.ready_for_goal = 2
            else:
                rospy.loginfo('Arm Move Failed')
                rospy.set_param('move_arm_status','failure')
                self.ready_for_goal = 2
        if self.ready_for_goal == 2:
            self.ready_for_goal = 0
            #self.cleanup()
            #rospy.signal_shutdown('Ending node.')

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == '__main__':
    move_arm_topic()
    rospy.spin()
