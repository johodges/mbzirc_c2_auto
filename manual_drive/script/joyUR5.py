#! /usr/bin/env python
import rospy, sys, roslib, rosparam, actionlib, time
import moveit_commander
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy, JointState
from moveit_msgs.msg import RobotState
from actionlib_msgs.msg import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionResult, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math

class joy_ur5():
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('joy_ur5', anonymous=False)
	
	self.min_pos = -2*3.1415
	self.max_pos = 2*3.1415
	self.step = 0.1
	self.max_acc = 3.0
    	self.last_pan = 0.0
	self.last_lift = 0.0
	self.last_elbow = 0.0
	self.last_wrist1 = 0.0
	self.last_wrist2 = 0.0
	self.last_wrist3 = 0.0

	# Subscribe to Joy and Publish to Base and Arm
	rospy.Subscriber("/joy", Joy, self.callback)
	rospy.Subscriber("/joint_states", JointState, self.joint_state)
	self.base_pub = rospy.Publisher('/joy_teleop/cmd_vel', Twist, queue_size=5)
	self.arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=5)
	
	# Initialize the move_group API
        #moveit_commander.roscpp_initialize(sys.argv)
        #robot = moveit_commander.RobotCommander()
        #scene = moveit_commander.PlanningSceneInterface()


    def callback(self, joy):
	# base control - press and hold LB and stick left
	#rospy.loginfo("Press and hold LB + stick left to move robot ... ")
	twist = Twist()
	c = joy.buttons[4]	# control signal - hold LB
	twist.linear.x = 4*c*joy.axes[1]	# move stick left up-down
	twist.angular.z = 2*c*joy.axes[0]	# move stick left left-right
	if c > 0:
	    self.base_pub.publish(twist)
	    print "Moving base: ", c, 20*c*joy.axes[1], 10*c*joy.axes[0]
	    #rospy.loginfo("Moving robot joystick ...")
	else:
	    #rospy.loginfo("No base teleop")
	    print "No base movement!"
	#rospy.loginfo("Press and hold RB + crosskey and stick right to linear move the arm ...")

	# arm control using buttons (X,A,B) and crosskey
	#rospy.loginfo("Hold RB plus X-A-B and crosskey for moving joint ...")

	

	# Get joint velocity from Joystick
	vel_pan = 3.15*joy.buttons[5]*joy.buttons[2]*joy.axes[6]	# press and hold X + crosskey up-down for shoulder pan
	#print vel_pan
	
	vel_lift = 3.15*joy.buttons[5]*joy.buttons[2]*joy.axes[7]	# press and hold X + crosskey left-right for shoulder lift
	#print vel_lift
	vel_elbow = 3.15*joy.buttons[5]*joy.buttons[0]*joy.axes[6]	# press and hold A + crosskey up-down for elbow
	#print vel_elbow
	vel_wrist1 = 3.15*joy.buttons[5]*joy.buttons[0]*joy.axes[7]	# press and hold A + crosskey left-right for wrist 1
	#print vel_wrist1
	vel_wrist2 = 3.15*joy.buttons[5]*joy.buttons[1]*joy.axes[6]	# press and hold B + crosskey up-down for wrist 2
	#print vel_wrist2
	vel_wrist3 = 3.15*joy.buttons[5]*joy.buttons[1]*joy.axes[7]	# press and hold B + crosskey left-right for wrist 3
	#print vel_wrist3
	
	if (vel_pan != 0):
	    active = True
	elif (vel_lift != 0):
	    active = True
	elif (vel_elbow <> 0):
	    active = True
	elif (vel_wrist1 <> 0):
	    active = True
	elif (vel_wrist2 <> 0):
	    active = True
	elif (vel_wrist3 <> 0):
	    active = True
	else:
	    active = False
 
	# Processing on arm
	arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

	# Load topic from param
	shoulder_pan_joint = 'ur5_arm_shoulder_pan_joint'
	shoulder_lift_joint = 'ur5_arm_shoulder_lift_joint'
	elbow_joint = 'ur5_arm_elbow_joint'
	wrist1_joint = 'ur5_arm_wrist_1_joint'
	wrist2_joint = 'ur5_arm_wrist_2_joint'
	wrist3_joint = 'ur5_arm_wrist_3_joint'

	# Measuring time:
	#dt = self.get_pressed_time(vel_wrist3)	
	#print self.state
	
	
 
	if (active):
	    try:
	        actual_pos = rospy.get_param('current_joint_state')
	    except:
	    	actual_pos = [0,0,0,0,0,0]
	    print "Actual position: ", actual_pos
	    travel = [0,0,0,0,0,0]
	    # Define velocity message
	    pan_vel = self.integrate(vel_pan, self.last_pan, self.max_acc, self.step)
	    pan_travel = self.step* (pan_vel + self.last_pan)/2.0;
	    pan = max(self.min_pos, min(self.max_pos, actual_pos[0] + pan_travel))
	    print "Pan params: ", pan_vel, pan_travel, pan
	    travel[0] = pan_travel 	

	    lift_vel = self.integrate(vel_lift, self.last_lift, self.max_acc, self.step)
	    lift_travel = self.step* (lift_vel + self.last_lift)/2.0;
	    lift = max(self.min_pos, min(self.max_pos, actual_pos[1] + lift_travel))
	    print "Lift params: ", lift_vel, lift_travel, lift
	    travel[1] = lift_travel

	    elbow_vel = self.integrate(vel_elbow, self.last_elbow, self.max_acc, self.step)
	    elbow_travel = self.step* (elbow_vel + self.last_elbow)/2.0;
	    elbow = max(self.min_pos, min(self.max_pos, actual_pos[2] + elbow_travel))
	    print "Elbow params: ", elbow_vel, elbow_travel, elbow
	    travel[2] = elbow_travel

	    wrist1_vel = self.integrate(vel_wrist1, self.last_wrist1, self.max_acc, self.step)	
	    wrist1_travel = self.step* (wrist1_vel + self.last_wrist1)/2.0;
	    wrist1 = max(self.min_pos, min(self.max_pos, actual_pos[3] + wrist1_travel))
	    print "Wrist1 params: ", wrist1_vel, wrist1_travel, wrist1
	    travel[3] = wrist1_travel

	    wrist2_vel = self.integrate(vel_wrist2, self.last_wrist2, self.max_acc, self.step)	
	    wrist2_travel = self.step* (wrist2_vel + self.last_wrist2)/2.0;
	    wrist2 = max(self.min_pos, min(self.max_pos, actual_pos[4] + wrist2_travel))
	    print "Wrist2 params: ", wrist2_vel, wrist2_travel, wrist2
	    travel[4] = wrist2_travel

	    wrist3_vel = self.integrate(vel_wrist3, self.last_wrist3, self.max_acc, self.step)
	    wrist3_travel = self.step* (wrist3_vel + self.last_wrist3)/2.0;
	    wrist3 = max(self.min_pos, min(self.max_pos, actual_pos[5] + wrist3_travel))
	    print "Wrist3 params: ", wrist3_vel, wrist3_travel, wrist3
	    travel[5] = wrist3_travel

	    # Getting goal and move the arm
	    goal = FollowJointTrajectoryGoal()
	    goal.trajectory.joint_names.append(shoulder_pan_joint)
	    goal.trajectory.joint_names.append(shoulder_lift_joint)
	    goal.trajectory.joint_names.append(elbow_joint)
	    goal.trajectory.joint_names.append(wrist1_joint)
	    goal.trajectory.joint_names.append(wrist2_joint)
	    goal.trajectory.joint_names.append(wrist3_joint)

	    p = JointTrajectoryPoint()
	    p.positions.append(pan)
	    p.positions.append(lift)
	    p.positions.append(elbow)
	    p.positions.append(wrist1)
	    p.positions.append(wrist2)
	    p.positions.append(wrist3)
   
	    p.velocities.append(pan_vel)
	    p.velocities.append(lift_vel)
	    p.velocities.append(elbow_vel)
	    p.velocities.append(wrist1_vel)
	    p.velocities.append(wrist2_vel)
	    p.velocities.append(wrist3_vel)
  
	    p.time_from_start = rospy.Duration(self.step)
	    goal.trajectory.points.append(p)
	    goal.goal_time_tolerance = rospy.Duration(0.0)
	    goal_pts = len(goal.trajectory.points)

	    # update current joint states
	    rospy.set_param('current_joint_state',goal.trajectory.points[goal_pts-1].positions)

	    arm_client.send_goal(goal)
	    #rospy.sleep(0.01)
	
	    # update based on actual timestep
	    #for i in range(6):
		#actual_pos[i]= actual_pos[i] + travel[i]
	    print "Updated position: ", actual_pos
	    d = rospy.Duration.from_sec(self.step)
	    pan_vel = self.integrate(vel_pan, self.last_pan, self.max_acc, d.to_sec())
	    pan_travel = d.to_sec()* (vel_pan + self.last_pan)/2.0;
	    pan = max(self.min_pos, min(self.max_pos, actual_pos[0] + pan_travel))
	    self.last_pan = pan_vel
	    print "Pan params update: ", pan_vel, pan_travel, pan
	    travel[0] = pan_travel
	
	    lift_vel = self.integrate(vel_lift, self.last_lift, self.max_acc, d.to_sec())
	    lift_travel = d.to_sec()* (vel_lift + self.last_lift)/2.0;
	    lift = max(self.min_pos, min(self.max_pos, actual_pos[1] + lift_travel))
	    self.last_lift = lift_vel
	    print "Lift params update: ", lift_vel, lift_travel, lift
	    travel[1] = lift_travel

	    elbow_vel = self.integrate(vel_elbow, self.last_elbow, self.max_acc, d.to_sec())
	    elbow_travel = d.to_sec()* (vel_elbow + self.last_elbow)/2.0;
	    elbow = max(self.min_pos, min(self.max_pos, actual_pos[2] + elbow_travel))
	    self.last_elbow = elbow_vel
	    print "Elbow params update: ", elbow_vel, elbow_travel, elbow
	    travel[2] = elbow_travel

	    wrist1_vel = self.integrate(vel_wrist1, self.last_wrist1, self.max_acc, d.to_sec())
	    wrist1_travel = d.to_sec()* (vel_wrist1 + self.last_wrist1)/2.0;
	    wrist1 = max(self.min_pos, min(self.max_pos, actual_pos[4] + wrist1_travel))
	    self.last_wrist1 = wrist1_vel
	    print "Wrist1 params update: ", wrist1_vel, wrist1_travel, wrist1
	    travel[3] = wrist1_travel
	
	    wrist2_vel = self.integrate(vel_wrist2, self.last_wrist2, self.max_acc, d.to_sec())
	    wrist2_travel = d.to_sec()* (vel_wrist2 + self.last_wrist2)/2.0;
	    wrist2 = max(self.min_pos, min(self.max_pos, actual_pos[5] + wrist2_travel))
	    self.last_wrist2 = wrist2_vel
	    print "Wrist2 params update: ", wrist2_vel, wrist2_travel, wrist2
	    travel[4] = wrist2_travel

	    wrist3_vel = self.integrate(vel_wrist3, self.last_wrist3, self.max_acc, d.to_sec())
	    wrist3_travel = d.to_sec()* (vel_wrist3 + self.last_wrist3)/2.0;
	    wrist3 = max(self.min_pos, min(self.max_pos, actual_pos[5] + wrist3_travel))
	    self.last_wrist3 = wrist3_vel
	    print "Wrrist3 params update: ", wrist3_vel, wrist3_travel, wrist3
	    travel[5] = wrist3_travel
	#break:

    def get_current_joint_state(self):
	try:
	    sz_joint = np.shape(self.state.name)
	    for i in range(sz_joint[0]):
	    	if (self.state.name[i] == shoulder_pan_joint):
		    actual_pos[i] = round(self.state.position[i],3)
	    	if (self.state.name[i] == shoulder_lift_joint):
		    actual_pos[i] = round(self.state.position[i],3)
	    	if (self.state.name[i] == elbow_joint):
		    actual_pos[i] = round(self.state.position[i],3)
	    	if (self.state.name[i] == wrist1_joint):
		    actual_pose[i] = round(self.state.position[i],3)
	    	if (self.state.name[i] == wrist2_joint):
		    actual_pos[i] = round(self.state.position[i],3)
	    	if (self.state.name[i] == wrist3_joint):
		    actual_pos[i] = round(self.state.position[i],3)
	except:
	    actual_pos = [0,0,0,0,0,0]	
	return actual_pos

    def joint_state(self, state):
	self.state = state
	return state
	
    def integrate(self,desired, present, max_rate, dt):
	if (desired > present):
	    return min(desired, present + max_rate*dt)
	else:
	    return max(desired, present - max_rate*dt)

    def joint_goal(self, jointname, pos, vel, dt):
 	goal = FollowJointTrajectoryGoal()
	goal.trajectory.joint_names.append(jointname)
	p = JointTrajectoryPoint()
	p.positions.push_back(pos)
	p.velocities.push_back(vel)
	p.time_from_start = rospy.Duration(dt)
	goal.trajectory.points.append(p)
	goal.goal_time_tolerance = rospy.Duration(0.0)
	return goal

    def get_pressed_time(self, vel):
	global tp1, tp2
	if vel != 0:
	    tp1 = rospy.Time.now()
	if vel == 0:    
	    tp2 = rospy.Time.now()
	    du = tp2 - tp1
	    print "Pressed time: ", du, du.to_sec()
	    dt = du.to_sec()
	    return dt
	
   

if __name__ == '__main__':
    try:
	joy_ur5()
	rospy.spin()
    except KeyboardInterrupt:
	print("Shuting down manual joy node")

