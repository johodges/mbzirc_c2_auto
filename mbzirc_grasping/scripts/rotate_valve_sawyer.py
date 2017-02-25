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
    arm = moveit_commander.MoveGroupCommander("right_arm")
    robot = moveit_commander.RobotCommander()
    jt = RobotState()
    jt.joint_state.header.frame_id = '/base'
    jt.joint_state.name = ['head_pan', 'right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    # Set the start state to the current state
    arm.set_start_state_to_current_state()

    joint_state = robot.get_current_state()
    joint_state_name = joint_state.joint_state.name
    joint_state_posi = joint_state.joint_state.position
    print joint_state_name, joint_state_posi
    arm.set_joint_value_target('head_pan', joint_state_posi[1])
    arm.set_joint_value_target('right_j0', joint_state_posi[0])
    arm.set_joint_value_target('right_j1', joint_state_posi[2])
    arm.set_joint_value_target('right_j2', joint_state_posi[3])
    arm.set_joint_value_target('right_j3', joint_state_posi[4])
    arm.set_joint_value_target('right_j4', joint_state_posi[5])
    arm.set_joint_value_target('right_j5', joint_state_posi[6])
    #arm.set_joint_value_target('right_j6', joint_state_posi[7])
    arm.set_joint_value_target('right_j6', 6.2)
    traj = arm.plan()
    arm.execute(traj)

    #arm.set_joint_value_target('ur5_arm_wrist_3_joint', -6.28)
    rospy.sleep(1)
    # Stop any current arm movement
    arm.stop()

    #rospy.set_param('smach_state','turnedValve')

    #Shut down MoveIt! cleanly
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

def main():
    move()

if __name__ == '__main__': main()
