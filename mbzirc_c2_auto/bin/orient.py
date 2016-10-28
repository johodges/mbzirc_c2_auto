#!/usr/bin/env python

""" orient.py - Version 1.0 2016-10-12

    This software uses a LIDAR scan to move around a box looking for wrenches.
    Made by Jonathan Hodges

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
import rospkg
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
from decimal import *
import tf

class orient():
    # A few key tasks are achieved in the initializer function:
    #     1. We connect to the move_base server in ROS
    #     2. We start the two ROS subscriber callback functions
    #     3. We initialize counters in the class to be shared by the various callback routines
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('orient', anonymous=True)
        
        # Enable shutdown in rospy (This is important so we cancel any move_base goals
        # when the node is killed)
        rospy.on_shutdown(self.shutdown) # Set rospy to execute a shutdown function when exiting
        
        # Establish number of loops through callback routine before we decide to resend
        # the move base goal since the robot is stuck
        self.stalled_threshold = rospy.get_param("~stalled_threshold", 100) # Loops before stall
        
        # Initialize counter variables
        self.old_bearing = 0
        self.ct3 = 0
        self.flag = 0
        self.wp = -1
        self.ct = 0
        self.ct_wrench = 0

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server and wait up to 60 seconds
        # for it to become available
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        self.goal = MoveBaseGoal()
        rospy.sleep(0.1)

        # Set up ROS subscriber callback routines
        rospy.Subscriber("/bearing", numpy_msg(Floats), self.callback, queue_size=1)
        rospy.Subscriber("/move_base/feedback", MoveBaseFeedback, self.callback_feedback, queue_size=1)
        rospy.Subscriber("/wrench_centroids", numpy_msg(Floats), self.callback_wrench, queue_size=1)

    # The shutdown function is used to cancel any move_base goals when this node is killed
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    # callback_feedback is used to store the feedback topic into the class to be
    # referenced by the other callback routines.
    def callback_feedback(self, data):
        self.feedback = data

    # callback_wrench is used to store the wrench topic into the class to be
    # referenced by the other callback routines.
    def callback_wrench(self, data):
        self.wrench = data.data

    # callback (bearing) is the primary function in this node. The general process is:
    #     1. Rotate husky until it is normal to the surface
    #     2. Check if wrenches are visible on the surface
    #     3. If no wrenches are visible, move the husky clockwise around the panel.
    # moves the husky until it is normal to the surface.
    #     4. Repeat 1-3 until wrenches are identified.
    #     5. Kill this node
    def callback(self, bearing):
        # We need some feedback from the move_base server to obtain our current
        # location. Since the move_base server does not publish feedback without an
        # active goal, we set an initial goal to get our position.
        if self.wp == -1:
            print "Setting initial state."
            self.goal.target_pose.header.frame_id = 'base_link'
            self.goal.target_pose.pose = Pose(Point(-0.5,0,0), Quaternion(0,0,0,1))
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            rospy.sleep(0.5)
            self.wp = self.wp+1
            self.state = self.move_base.get_state()
        else:

            # We check to see if we are bouncing between two angle increments, if
            # yes increment counter, otherwise set to 0.
            if bearing.data[0] != self.old_bearing:
                self.ct3 = self.ct3+1
            else:
                self.ct3 = 0
            
            # Check if we are currently trying to rotate around the panel
            if self.flag == 0:
                # Extract points of interest from the /bearing topic:
                #     point A = median point of laser scan
                #     point B = point of laser scan closest to the robot
                #     mn/mx = minimum/maximum extents of the object
                xA = bearing.data[1]
                yA = bearing.data[2]
                xB = bearing.data[3]
                yB = bearing.data[4]
                xmn = bearing.data[5]
                xmx = bearing.data[6]
                ymn = bearing.data[7]
                ymx = bearing.data[8]

                # Extract the current pose of the robot in the global reference from
                # the /feedback topic:
                x0 = self.feedback.feedback.base_position.pose.position.x
                y0 = self.feedback.feedback.base_position.pose.position.y
                z0 = self.feedback.feedback.base_position.pose.position.z
                X0 = self.feedback.feedback.base_position.pose.orientation.x
                Y0 = self.feedback.feedback.base_position.pose.orientation.y
                Z0 = self.feedback.feedback.base_position.pose.orientation.z
                W0 = self.feedback.feedback.base_position.pose.orientation.w
                
                # Reset the robot stuck counter
                self.ct = 0

                # If we are not convinced we are normal to the surface, rotate the robot
                # towards the surface. Otherwise, define a target rotating CW around the box.
                if self.ct3 < 5:
                    q = tf.transformations.quaternion_from_euler(0,0,bearing.data[0])
                    self.goal.target_pose.pose = Pose(Point(0,0,0), Quaternion(q[0],q[1],q[2],q[3]))
                    #print self.goal.target_pose.pose
                    self.flag = 0
                    self.goal.target_pose.header.frame_id = 'base_link'
                else:
                    print "Let's circle around!"
                    # Convert quaternion angle to euler angles
                    euler = tf.transformations.euler_from_quaternion([X0,Y0,Z0,W0])
                    roll = euler[0]
                    pitch = euler[1]
                    yaw = euler[2]
                    theta = yaw

                    # Define rotation matrix
                    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
                    print "Rotation Matrix:", R

                    # Calculate the object location in local coordinate system
                    x_loc = ((xmx-xmn)/2)+xmn
                    y_loc = ((ymx-ymn)/2)+ymn
                    print "Object in local coord and local sys:", x_loc, y_loc, Z0
                    obj_loc = np.array([[x_loc],[y_loc]])
                    print "ymx, ymn:", ymx, ymn

                    # Define the target location in the local coordinate system
                    tar_loc = np.array([[xmn+1],[ymx+2]])#y_loc+x_loc+ymx+1]])
                    print "Target loc in local coord and local sys:", tar_loc

                    # Convert local object and target locations to global coordinate system
                    obj_glo = np.dot(R,obj_loc)
                    tar_glo = np.dot(R,tar_loc)
                    x_obj_glo = obj_glo[0]+x0
                    y_obj_glo = obj_glo[1]+y0
                    x_tar_glo = x0+tar_glo[0]
                    y_tar_glo = y0+tar_glo[1]
                    print "Object in global coord and global sys:", x_obj_glo, y_obj_glo
                    print "Target in global coord and global sys:", x_tar_glo, y_tar_glo

                    # Pause for a few seconds to allow user to cancel if needed
                    rospy.sleep(5) 
                    
                    # The path planner likes to try and run into the object. We force the
                    # robot to move in a specific direction initially to mitigate this.
                    print "Rotating to make path better."
                    q = tf.transformations.quaternion_from_euler(0,0,yaw-1.57)
                    loc = Pose(Point(x_tar_glo,y_tar_glo,0), Quaternion(q[0],q[1],q[2],q[3]))
                    q = tf.transformations.quaternion_from_euler(0,0,1.57)
                    self.goal.target_pose.pose = Pose(Point(0,0,0),Quaternion(q[0],q[1],q[2],q[3]))
                    self.goal.target_pose.header.frame_id = 'base_link'
                    self.move_base.send_goal(self.goal)
                    rospy.sleep(1)
                    self.ct_move = 0

                    # Loop until goal is reached. If stalled, resend goal
                    while self.move_base.get_state() != 3:
                        if self.ct_move > self.stalled_threshold:
                            self.move_base.send_goal(self.goal)
                            self.ct_move = 0
                        self.ct_move = self.ct_move + 1
                        rospy.sleep(0.1)
                    print "Done rotating!"
                    print "Moving around the box a bit to give clearance."
                    q = tf.transformations.quaternion_from_euler(0,0,-1.57)
                    self.goal.target_pose.pose = Pose(Point(2,0,0),Quaternion(q[0],q[1],q[2],q[3]))
                    self.move_base.send_goal(self.goal)
                    rospy.sleep(1)
                    self.ct_move = 0

                    # Loop until goal is reached. If stalled, resend goal
                    while self.move_base.get_state() != 3:
                        if self.ct_move > self.stalled_threshold:
                            self.move_base.send_goal(self.goal)
                            self.ct_move = 0
                        self.ct_move = self.ct_move + 1
                        rospy.sleep(0.1)
                    print "Done getting clearance."

                    # Move to target location
                    print "Goal location in global frame:", x_tar_glo, y_tar_glo
                    print abs(xmx-xmn)+xmn, abs(ymx-ymn)+ymn+xmn
                    self.goal.target_pose.pose = loc
                    self.goal.target_pose.header.frame_id = 'odom'
                    self.move_base.send_goal(self.goal)
                    self.ct3 = 0
                    rospy.sleep(1)
                    self.ct_move = 0

                    # Loop until goal is reached. If stalled, resend goal
                    while self.move_base.get_state() != 3:
                        if self.ct_move > self.stalled_threshold:
                            self.move_base.send_goal(self.goal)
                            self.ct_move = 0
                        self.ct_move = self.ct_move + 1
                        rospy.sleep(0.1)
                self.goal.target_pose.header.stamp = rospy.Time.now()
            if self.flag == 0:
                self.move_base.send_goal(self.goal)
            else:
                #Check if we reached target location. If yes, start the normalizing process.
                if self.move_base.get_state() == 3:
                    self.flag = 0
            
            # Store old heading information to check if we are normal to surface
            self.old_bearing = bearing.data[0]
            rospy.sleep(1)
            state = self.move_base.get_state()

            # If we are normal, set flag to move around the box
            if self.ct3 > 4:
                self.flag = 1
                rospy.sleep(5)
            
            # If we have not reached our goal (presumably stuck) resend the goal
            if self.ct > self.stalled_threshold:
                self.move_base.send_goal(self.goal)
                self.ct = 0
            else:
                self.ct = self.ct+1

            # Check if we see wrenches
            if np.shape(self.wrench)[0] > 6:
                print "We found wrenches!"
                self.ct_wrench = self.ct_wrench+1
                self.ct3 = 0
                # Make sure we saw wrenches 5 times through the loop (reduce false positives)
                if self.ct_wrench > 5:
                    print "We are confident these are the wrenches we are looking for!"
                    rospy.sleep(1)
                    rospy.signal_shutdown('Ending node.')
            else:
                self.ct_wrench = 0

if __name__ == '__main__':
    try:
        orient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("orient finished.")

