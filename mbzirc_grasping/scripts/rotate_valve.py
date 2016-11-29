#!/usr/bin/env python
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import moveit_commander
from moveit_msgs.msg import RobotState

def move():
	# Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        arm = moveit_commander.MoveGroupCommander("ur5_arm")

	try:
            cjs = rospy.get_param('current_joint_state')
        except:
            cjs = [0,0,0,0,0,0]
        jt = RobotState()
        jt.joint_state.header.frame_id = '/base_link'
        jt.joint_state.name = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel', 'ur5_arm_shoulder_pan_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_elbow_joint', 'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint', 'left_tip_hinge', 'right_tip_hinge']
        jt.joint_state.position = [0,0,0,0,cjs[0],cjs[1],cjs[2],cjs[3],cjs[4],cjs[5],0,0]

        # Set the start state to the current state
        arm.set_start_state(jt)

	    arm.set_joint_value_target('ur5_arm_shoulder_pan_joint', cjs[0])
    	arm.set_joint_value_target('ur5_arm_shoulder_lift_joint', cjs[1])
    	arm.set_joint_value_target('ur5_arm_elbow_joint', cjs[2])
    	arm.set_joint_value_target('ur5_arm_wrist_1_joint', cjs[3])
    	arm.set_joint_value_target('ur5_arm_wrist_2_joint', cjs[4])
	    arm.set_joint_value_target('ur5_arm_wrist_3_joint', 0.0) #3.14159)

	    traj = arm.plan()

	    arm.execute(traj)

	    rospy.sleep(5.0)
        jt.joint_state.position = [0,0,0,0,cjs[0],cjs[1],cjs[2],cjs[3],cjs[4],0.0,0,0]
        arm.set_joint_value_target('ur5_arm_wrist_3_joint', -6.28)

	    traj = arm.plan()

	    arm.execute(traj)

	    rospy.sleep(5.0)

        # Stop any current arm movement
        arm.stop()

        #Shut down MoveIt! cleanly
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

def main():
	move()

if __name__ == '__main__': main()
