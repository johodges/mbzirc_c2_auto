#!/usr/bin/env python

import roslib;
import rospy
import smach
import smach_ros
import time
import os
import subprocess
from geometry_msgs.msg import Twist
import math
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

JOINT_NAMES = ['ur5_arm_shoulder_pan_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_elbow_joint', 'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint']

# define states
class FindBoard(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['atBoard'])

    def execute(self, userdata):
        rospy.loginfo('Searching for board')
	
	a = subprocess.Popen("rosrun mbzirc_c2_auto findbox.py", shell=True)
	b = subprocess.Popen("rosrun mbzirc_c2_auto autonomous.py", shell=True)

	b.wait()
	rospy.loginfo('Searching for board')
	rospy.loginfo('Searching for board')
	rospy.loginfo('Searching for board')
	rospy.loginfo('Searching for board')
	rospy.loginfo('Searching for board')
	rospy.loginfo('Searching for board')
	rospy.loginfo('Searching for board')
	rospy.loginfo('Searching for board')
	a.kill()

        return 'atBoard'

class Orient(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['oriented'])

    def execute(self, userdata):
        rospy.loginfo('Orienting')
        
	c = subprocess.Popen("rosrun mbzirc_c2_auto orient.py", shell=True)
	d = subprocess.Popen("rosrun mbzirc_c2_auto orient_scan.py", shell=True)
	e = subprocess.Popen("rosrun mbzirc_c2_auto wrench_detect.py", shell=True)

	c.wait()
        rospy.loginfo('Orienting')
        rospy.loginfo('Orienting')
        rospy.loginfo('Orienting')
        rospy.loginfo('Orienting')
        rospy.loginfo('Orienting')
        rospy.loginfo('Orienting')
	d.kill()
	e.kill()

	#twist = Twist()
	#twist.linear.x = 0.5
	#twist.linear.y = 0
        #twist.linear.z = 0
        #twist.angular.x = 0
        #twist.angular.y = 0
        #twist.angular.z = 0
	#pub = rospyPublisher = rospy.Publisher("/joy_teleop/cmd_vel", Twist, queue_size=10)
	#for x in range(0, 150000):
	#	pub.publish(twist)

        return 'oriented'

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['graspedWrench'])

    def execute(self, userdata):
        rospy.loginfo('Grasping wrench')

	client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

	q = [-2.13237, -1.78783, -1.91733, -2.57803, -0.561577 - (math.pi / 2.0), 3.14159]
	print "Waiting for ur5_arm server..."
        client.wait_for_server()
        print "Connected to ur5_arm server"
        g = FollowJointTrajectoryGoal()
	g.trajectory = JointTrajectory()
    	g.trajectory.joint_names = JOINT_NAMES
    	g.trajectory.points = [JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
	client.send_goal(g)
	try:
		client.wait_for_result()
	except KeyboardInterrupt:
		client.cancel_goal()
		raise

        return 'graspedWrench'

class UseWrench(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Opening valve')

	client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

	q = [-0.94455, -1.489, -2.24215, -2.55204, 0.626246 - (math.pi / 2.0), 3.14159]
	print "Waiting for ur5_arm server..."
        client.wait_for_server()
        print "Connected to ur5_arm server"
        g = FollowJointTrajectoryGoal()
	g.trajectory = JointTrajectory()
    	g.trajectory.joint_names = JOINT_NAMES
    	g.trajectory.points = [JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
	client.send_goal(g)
	try:
		client.wait_for_result()
	except KeyboardInterrupt:
		client.cancel_goal()
		raise
        
	return 'succeeded'

# main
def main():
    rospy.init_node('mbzirc_simulation_state_machine', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FINDBOARD', FindBoard(), 
                               transitions={'atBoard':'ORIENT'})

        smach.StateMachine.add('ORIENT', Orient(), 
                               transitions={'oriented':'GRASP'})

        smach.StateMachine.add('GRASP', Grasp(), 
                               transitions={'graspedWrench':'USEWRENCH'})

        smach.StateMachine.add('USEWRENCH', UseWrench(), 
                               transitions={'succeeded':'success'})

    # Create the introspection server
    sis = smach_ros.IntrospectionServer('mbzirc_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
