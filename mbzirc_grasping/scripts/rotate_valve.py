#!/usr/bin/env python
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import moveit_commander

def move():
	# Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        arm = moveit_commander.MoveGroupCommander("ur5_arm")

	arm.set_joint_value_target('ur5_arm_wrist_3_joint', 3.14159)

	traj = arm.plan()

	arm.execute(traj)

def main():
	move()

if __name__ == '__main__': main()
