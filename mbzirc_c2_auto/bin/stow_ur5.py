#!/usr/bin/env python
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['ur5_arm_shoulder_pan_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_elbow_joint',
               'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint']

#JOINT_NAMES = ['head_pan', 'right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

#Q1 = [-0.06946574348407708, 0.0740005691912522, 8.811494629235028e-05, 0.00012994800542465157, -6.347990724009378e-05, -0.00012303364955990048, 0.015344299965057928, -6.530714214443398]
#Q2 = [-0.06946574348407708, 0.0740005691912522, 1.0, 0.00012994800542465157, -6.347990724009378e-05, -0.00012303364955990048, 0.015344299965057928, -6.530714214443398]
#Q3 = [-0.06946574348407708, 0.0740005691912522, 1.0, 1.0, -6.347990724009378e-05, -0.00012303364955990048, 0.015344299965057928, -6.530714214443398]
#JOINT_NAMES = ['front_left_wheel', 'front_right_wheel', 'head_pan', 'left_tip_hinge', 'rear_left_wheel', 'rear_right_wheel', 'right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6', 'right_tip_hinge']


Q1 = [1.5707,-1.57,0.0,0.0,0.0,0.0]
Q2 = [1.5707,-1.57,0.0,-2.7925,-1.5707,0.0]
Q3 = [1.5707,-2.6927,2.6927,-3.1415,-1.5707,0.0]

client = None

def move():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(6.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    client.wait_for_result()

def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    d = 2.0
    g.trajectory.points = []
    for i in range(10):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]

    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for ur5_arm server..."
        client.wait_for_server()
        print "Connected to ur5_arm server"
        move()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
