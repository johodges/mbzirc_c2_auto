#!/usr/bin/env python
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import moveit_commander
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class rotate_valve():
    def __init__(self):
        rospy.init_node('rotate_valve', anonymous=False)
        rospy.Subscriber("/joint_states",JointState, self.cb_joint, queue_size=1)
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander("ur5_arm")
        self.jt = RobotState()
        self.jt.joint_state.header.frame_id = '/base_link'

        self.jt.joint_state.name = ['ur5_arm_shoulder_pan_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_elbow_joint', 'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint']
        self.arm.set_start_state_to_current_state()

    def cb_joint(self, data):
        print "DATA: "
        cjs = data.position
        self.arm.set_joint_value_target('ur5_arm_shoulder_pan_joint', cjs[0])
        self.arm.set_joint_value_target('ur5_arm_shoulder_lift_joint', cjs[1])
        self.arm.set_joint_value_target('ur5_arm_elbow_joint', cjs[2])
        self.arm.set_joint_value_target('ur5_arm_wrist_1_joint', cjs[3])
        self.arm.set_joint_value_target('ur5_arm_wrist_2_joint', cjs[4])
        self.arm.set_joint_value_target('ur5_arm_wrist_3_joint', cjs[5]+2*3.14) #3.14159)
        traj = self.arm.plan()
        self.arm.execute(traj)

        rospy.sleep(5.0)
        # Stop any current arm movement
        self.arm.stop()

        rospy.set_param('smach_state','turnedValve')

        # Shut down MoveIt! cleanly
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

def main():
    rotate_valve()
    rospy.spin()

if __name__ == '__main__': main()
