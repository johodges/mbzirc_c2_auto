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
import math

class drive2valve():
    # A few key tasks are achieved in the initializer function:
    #     1. We connect to the move_base server in ROS
    #     2. We start the two ROS subscriber callback functions
    #     3. We initialize counters in the class to be shared by the various callback routines
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('drive2valve', anonymous=True)

        # Enable shutdown in rospy (This is important so we cancel any move_base goals
        # when the node is killed)
        rospy.on_shutdown(self.shutdown) # Set rospy to execute a shutdown function when exiting

        # Establish number of loops through callback routine before we decide to resend
        # the move base goal since the robot is stuck
        self.stalled_threshold = rospy.get_param("~stalled_threshold", 500) # Loops before stall

        # Initialize counter variables
        self.old_bearing = 0
        self.ct3 = 0
        self.flag = 0
        self.wp = -1
        self.ct = 0
        self.ct_wrench = 0
        self.off = 0

        # Store camera parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080

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

        # Set up tf listener
        self.tftree = tf.TransformListener()

        # Set up ROS subscriber callback routines
        rospy.Subscriber("/bearing", numpy_msg(Floats), self.callback, queue_size=1)
        rospy.Subscriber("/move_base/feedback", MoveBaseFeedback, self.callback_feedback, queue_size=1)
        rospy.Subscriber("/valve", numpy_msg(Floats), self.callback_v_c, queue_size=1)

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

    # callback_v_c is used to store the valve center topic into the class to be
    # referenced by the other callback routines.
    def callback_v_c(self, data):
        self.v_c = data.data

    # callback (bearing) is the primary function in this node. The general process is:
    #     1. Rotate husky until it is normal to the surface
    #     2. Check if wrenches are visible on the surface
    #     3. If no wrenches are visible, move the husky clockwise around the panel.
    # moves the husky until it is normal to the surface.
    #     4. Repeat 1-3 until wrenches are identified.
    #     5. Kill this node
    def callback(self, bearing):

        # wait_for_finish - this subroutine waits for a goal to finish before returning
        def wait_for_finish(stalled_threshold):
            self.ct_move = 0
            rospy.sleep(1)
            while self.move_base.get_state() != 3:
                if self.ct_move > stalled_threshold:
                    print "We are stuck! Cancelling current goal and moving backwards 0.5m."
                    self.move_base.cancel_goal()
                    rospy.sleep(1)
                    back_it_up(-0.25,0.5)
                    rospy.sleep(0.5)
                    rot_cmd(0.25,0.25)
                    print "Resending goal."
                    self.move_base.send_goal(self.goal)
                    rospy.sleep(1)
                    self.ct_move = 0
                self.ct_move = self.ct_move + 1
                rospy.sleep(0.1)
            return None

        # tar_in_global - this subroutine converts local coordinate target to global coordinates
        def tar_in_global(tar_loc):
            tar_glo = np.dot(self.R,tar_loc)
            self.x_tar_glo = self.x0+tar_glo[0]
            self.y_tar_glo = self.y0+tar_glo[1]
            return None

        # back_it_up - this subroutine moves the husky forward or backward a distance by manually
        # controlling the wheels
        def back_it_up(ve,dist_to_move):
            sleep_time = 0.1
            time_to_move = abs(dist_to_move/ve)
            twist = Twist()
            twist.linear.x = ve
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            twi_pub = rospy.Publisher("/joy_teleop/cmd_vel", Twist, queue_size=10)
            self.ct_move = 0
            while self.ct_move*sleep_time < time_to_move:
                twi_pub.publish(twist)
                self.ct_move = self.ct_move+1
                rospy.sleep(sleep_time)
            return None

        # rot_cmd - this subroutine rotates the husky 90 degrees by controlling the wheels manually
        def rot_cmd(ve,dist_to_move):
            sleep_time = 0.1
            time_to_move = abs(dist_to_move/ve)
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = ve
            twi_pub = rospy.Publisher("/joy_teleop/cmd_vel", Twist, queue_size=10)
            self.ct_move = 0
            while self.ct_move*sleep_time < time_to_move:
                twi_pub.publish(twist)
                self.ct_move = self.ct_move+1
                rospy.sleep(sleep_time)
            return None

        # update_rot - this subroutine updates the feedback locations
        def update_rot():
            # Extract the current pose of the robot in the global reference from
            # the /feedback topic:
            self.x0 = self.feedback.feedback.base_position.pose.position.x
            self.y0 = self.feedback.feedback.base_position.pose.position.y
            self.z0 = self.feedback.feedback.base_position.pose.position.z
            self.X0 = self.feedback.feedback.base_position.pose.orientation.x
            self.Y0 = self.feedback.feedback.base_position.pose.orientation.y
            self.Z0 = self.feedback.feedback.base_position.pose.orientation.z
            self.W0 = self.feedback.feedback.base_position.pose.orientation.w
            # Convert quaternion angle to euler angles
            euler = tf.transformations.euler_from_quaternion([self.X0,self.Y0,self.Z0,self.W0])
            self.roll = euler[0]
            self.pitch = euler[1]
            self.yaw = euler[2]
            self.theta = self.yaw
            # Define rotation matrix
            self.R = np.array([[np.cos(self.theta),-np.sin(self.theta)],[np.sin(self.theta),np.cos(self.theta)]])
            return None

        # We need some feedback from the move_base server to obtain our current
        # location. Since the move_base server does not publish feedback without an
        # active goal, we set an initial goal to get our position.
        if self.wp == -1:
            rospy.loginfo("Moving backward 1m to make move to valve easier.")
            back_it_up(-0.25,0.5)
            self.goal.target_pose.header.frame_id = 'base_link'
            self.goal.target_pose.pose = Pose(Point(-0.5,0,0), Quaternion(0,0,0,1))
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            rospy.sleep(1)
            self.wp = self.wp+1
        else:
            update_rot()
            rospy.loginfo("Setting initial estimate of valve location.")
            valve = rospy.get_param('valve')
            ugv_pos = rospy.get_param('ugv_position')
            val_loc = [valve[0]-1,valve[1]]
            val_glo = np.dot(self.R,val_loc)
            self.x_val_glo = val_glo[0]+ugv_pos[0]
            self.y_val_glo = val_glo[1]+ugv_pos[1]
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose = Pose(Point(self.x_val_glo,self.y_val_glo,0), Quaternion(ugv_pos[3],ugv_pos[4],ugv_pos[5],ugv_pos[6]))
            self.goal.target_pose.header.frame_id = 'odom'
            self.move_base.send_goal(self.goal)
            print "Moving to intial estimate of valve location."
            wait_for_finish(100)
            self.flag = 1
            
            if self.flag == 1:
                rospy.sleep(1)
                self.ct_move = 0
                wait_for_finish(100)
                valve = self.v_c[0]
                vw_c = valve #(valve+wrenc)/2
                vw_t = 960
                vw_off = (vw_c-vw_t)
                update_rot()

                # Check if we are centered between valve and wrenches
                if abs(vw_off) <= 50:
                    print "Victory!"
                    xA = bearing.data[1]
                    yA = bearing.data[2]
                    xB = bearing.data[3]
                    yB = bearing.data[4]
                    xmn = bearing.data[5]
                    xmx = bearing.data[6]
                    ymn = bearing.data[7]
                    ymx = bearing.data[8]

                    # Calculate the object location in local coordinate system
                    x_loc = ((xmx-xmn)/2)+xmn
                    y_loc = ((ymx-ymn)/2)+ymn
                    print "Object in local coord and local sys:", x_loc, y_loc, self.Z0
                    obj_loc = np.array([[x_loc],[y_loc]])
                    po = 1
                    back_it_up(0.25,(x_loc-po))
                    rospy.set_param('smach_state','valvepos')
                    rospy.signal_shutdown('Ending node.')
                else:
                    back_it_up(-0.25,1)

                    if vw_off < 0:
                        di = -0.1
                    else:
                        di = 0.1
                    self.goal.target_pose.pose = Pose(Point(self.x0+di*np.sin(self.yaw),self.y0-di*np.cos(self.yaw),0), Quaternion(self.X0,self.Y0,self.Z0,self.W0))
                    self.goal.target_pose.header.frame_id = 'odom'
                    self.move_base.send_goal(self.goal)
                    print "Moving forward 1m and to the left-right 0.2m"
                    wait_for_finish(100)

if __name__ == '__main__':
    try:
        drive2valve()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("drive2valve killed.")

