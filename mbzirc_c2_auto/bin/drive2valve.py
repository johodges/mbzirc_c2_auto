#!/usr/bin/env python

"""drive2valve.py - Version 1.0 2016-10-12
Author: Jonathan Hodges

This software drives the husky from the wrench position to the valve position.

Subscribers:
    /bearing: array containing [angle,x_med,xmn,xmx,ymn,ymx,target_x,target_y]
    /move_base/feedback: Topic containing current position and orientation of
        UGV from move_base server
    /tf: TF tree
    /valve: Topic containing estimated center of valve (I don't think this is
        currently in use)

Publishers:
    /joy_teleop/cmd_vel: topic to manually move the robot
    /move_base/goal: goal sent to the move_base node in ROS

Parameters:
    valve: valve location in global coordinates
    wrench: wrench location in global coordinates
    ee_position: current position of end effector in base_link coordinates
    stow_position: end effector location in stow position
    current_joint_state: initalize joint states before moving arm
    ugv_position: current ugv position in global coordinates
    smach_state: status for state machine

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html
    
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import tf

class drive2valve():
    """This software drives the husky from the wrench position to the valve
    position.

    Attributes:
        camera_fov_h: Horizontal field of view of camera in radians
        camera_fov_v: Vertical field of view of camera in radians
        camera_pix_h: Horizontal field of view of camera in pixels
        camera_pix_v: Vertical field of view of camera in pixels
        ct_move: manually movement counter
        fake_smach: flag to transition to different nodes within this node
        feedback: position and orientation feedback from the move_base server
        goal: move_base goal
        move_base: move_base action server
        rest_time: default sleep time
        stalled_threshold: threshold for stalled UGV
        tftree: TF tree
        twi_pub: Publisher to manually control the robot 
        wp: flag to get a goal at the start
        v_c: valve center
        x_tar_glo: target location x coordinate in global coordinates
        y_tar_glo: target location y coordinate in global coordinates
        x0: x position from feedback from move_base server
        y0: y position from feedback from move_base server
        z0: z position from feedback from move_base server
        X0: X quaternion from feedback from move_base server
        Y0: Y quaternion from feedback from move_base server
        Z0: Z quaternion from feedback from move_base server
        W0: W quaternion from feedback from move_base server
        roll: x orientation from feedback from move_base server
        pitch: y orientation from feedback from move_base server
        yaw: z orientation from feedback from move_base server
        theta: z orientation from feedback from move_base server
        R: rotation matrix to convert local system to global system

    Subscribers:
        /bearing: array containing
            [angle,x_med,xmn,xmx,ymn,ymx,target_x,target_y]
        /move_base/feedback: Topic containing current position and orientation
            of UGV from move_base server
        /tf: TF tree
        /valve: Topic containing estimated center of valve (I don't think this
            is currently in use)

    Publishers:
        /joy_teleop/cmd_vel: topic to manually move the robot
        /move_base/goal: goal sent to the move_base node in ROS

    Parameters:
        valve: valve location in global coordinates
        wrench: wrench location in global coordinates
        ee_position: current position of end effector in base_link coordinates
        stow_position: end effector location in stow position
        current_joint_state: initalize joint states before moving arm
        ugv_position: current ugv position in global coordinates
        smach_state: status for state machine

    """

    def __init__(self):
        """This initializes the various attributes in the class

        A few key tasks are achieved in the initializer function:
            1. We connect to the move_base server in ROS
            2. We start the ROS publishers and subscribers
            3. We initialize shared parameters

        These are the possible returns from move_base.get_state() function.
        Included for reference:
        goal_states: ['PENDING', 'ACTIVE', 'PREEMPTED','SUCCEEDED', 'ABORTED',
                      'REJECTED','PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

        """
        # Establish ROS node
        rospy.init_node('drive2valve', anonymous=True) # Name this node
        rospy.on_shutdown(self.shutdown) # Enable shutdown in rospy

        # Initialize parameters
        self.rest_time = 0.1            # Minimum pause at each location
        self.stalled_threshold = 500    # Loops before stall
        self.fake_smach = 0             # Flag to transition to diff actions
        self.wp = -1                    # Set to -1 to get goal at start

        # Hardware Parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080

        # Establish publishers and subscribers
        self.twi_pub = rospy.Publisher("/joy_teleop/cmd_vel", Twist,
            queue_size=5)
        self.tftree = tf.TransformListener() # Set up tf listener
        rospy.Subscriber("/bearing", numpy_msg(Floats), self.callback,
            queue_size=1)
        rospy.Subscriber("/move_base/feedback", MoveBaseFeedback,
            self.callback_feedback, queue_size=1)
        rospy.Subscriber("/valve", numpy_msg(Floats), self.callback_v_c,
            queue_size=1)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base",
            MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        self.goal = MoveBaseGoal()

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting drive to valve.")

        rospy.sleep(self.rest_time)

    def shutdown(self):
        """This subroutine runs when the drive2valve node shutdown. It is
        important to cancel any move_base goals prior to ending the node so
        the robot does not keep moving after this node dies.
        """
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(self.rest_time)
        self.twi_pub.publish(Twist())
        rospy.sleep(self.rest_time)

    def callback_feedback(self, data):
        """This callback is used to store the feedback topic into the class to
        be referenced by the other callback routines.
        """
        self.feedback = data

    def callback_v_c(self, data):
        """This callback is used to store the valve center topic into the class
        to be referenced by the other callback routines.
        """
        self.v_c = data.data

    def callback(self, bearing):
        """This callback drives the UGV in front of the valve.

        The general process is:
            1. Move to the rough estimate of valve center
            2. Visual servo to center on valve, and then move forward
            3. Kill this node

        """

        def wait_for_finish(stalled_threshold):
            """This subroutine waits for a goal to finish before returning
            """
            self.ct_move = 0
            rospy.sleep(self.rest_time)
            while self.move_base.get_state() != 3:
                if self.ct_move > stalled_threshold:
                    rospy.loginfo("We are stuck! Entering stuck routine.")
                    self.move_base.cancel_goal()
                    rospy.sleep(0.1)
                    back_it_up(-0.25,0.5)
                    rospy.sleep(0.1)
                    rot_cmd(0.25,0.25)
                    self.move_base.send_goal(self.goal)
                    rospy.sleep(self.rest_time)
                    self.ct_move = 0
                self.ct_move = self.ct_move + 1
                rospy.sleep(self.rest_time)
            return None

        def tar_in_global(tar_loc):
            """This subroutine converts local coordinate target to global
            coordinates.
            """
            tar_glo = np.dot(self.R,tar_loc)
            self.x_tar_glo = self.x0+tar_glo[0]
            self.y_tar_glo = self.y0+tar_glo[1]
            return None

        def back_it_up(ve,dist_to_move):
            """This subroutine manually moves the husky forward or backward
            a fixed amount at a fixed velocity by bypassing the move_base
            package and sending a signal directly to the wheels.
                
            This subroutine is likely to be replaced by:
                file:
                    robot_mv_cmds
                    subroutine: move_UGV_vel(lv,av,dist_to_move)
            """ 
            time_to_move = abs(dist_to_move/ve)
            twist = Twist()
            twist.linear.x = ve
            self.ct_move = 0
            while self.ct_move*self.rest_time < time_to_move:
                self.twi_pub.publish(twist)
                self.ct_move = self.ct_move+1
                rospy.sleep(self.rest_time)
            return None

        def rot_cmd(ve,dist_to_move):
            """This subroutine manually rotates the husky along the z-axis a
            fixed amount at a fixed velocity by bypassing the move_base package
            and sending a signal directly to the wheels.

            This subroutine is likely to be replaced by:
                file:
                    robot_mv_cmds
                    subroutine: move_UGV_vel(lv,av,dist_to_move)
            """
            time_to_move = abs(dist_to_move/ve)
            twist = Twist()
            twist.angular.z = ve
            self.ct_move = 0
            while self.ct_move*self.rest_time < time_to_move:
                self.twi_pub.publish(twist)
                self.ct_move = self.ct_move+1
                rospy.sleep(self.rest_time)
            return None

        def update_rot():
            """This subroutine updates the feedback locations in the global
            reference frame from the /feedback topic.
            """
            self.x0 = self.feedback.feedback.base_position.pose.position.x
            self.y0 = self.feedback.feedback.base_position.pose.position.y
            self.z0 = self.feedback.feedback.base_position.pose.position.z
            self.X0 = self.feedback.feedback.base_position.pose.orientation.x
            self.Y0 = self.feedback.feedback.base_position.pose.orientation.y
            self.Z0 = self.feedback.feedback.base_position.pose.orientation.z
            self.W0 = self.feedback.feedback.base_position.pose.orientation.w
            # Convert quaternion angle to euler angles
            euler = tf.transformations.euler_from_quaternion([self.X0,self.Y0,
                self.Z0,self.W0])
            self.roll = euler[0]
            self.pitch = euler[1]
            self.yaw = euler[2]
            self.theta = self.yaw
            # Define rotation matrix
            self.R = np.array([[np.cos(self.theta),-np.sin(self.theta)],
                [np.sin(self.theta),np.cos(self.theta)]])
            return None

        if self.wp == -1:
            """We need some feedback from the move_base server to obtain our
            current location. Since the move_base server does not publish
            feedback without an active goal, we set an initial goal to get our
            position.

            Ideally this would be replaced by pulling the position of the UGV
            from a different feedback topic or the TF tree.
            """
            rospy.loginfo("Moving backward 2m to make move to valve easier.")
            back_it_up(-0.25,2)
            self.goal.target_pose.header.frame_id = 'base_link'
            self.goal.target_pose.pose = Pose(Point(-0.5,0,0),
                Quaternion(0,0,0,1))
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            rospy.sleep(self.rest_time*10)
            self.wp = self.wp+1
        else:
            update_rot()
            """Extract points of interest from the /bearing topic:
            point A: median point of laser scan
            point C: unused in this routine
            mn/mx: minimum/maximum extents of the object
            """
            ang = bearing.data[0]
            xA = bearing.data[1]+0.025
            xmn = bearing.data[2]
            xmx = bearing.data[3]
            ymn = bearing.data[4]
            ymx = bearing.data[5]
            xC = bearing.data[6]
            yC = bearing.data[7]

            if self.fake_smach == 2:
                """This node estimates the valve location from the camera
                frame and kills this node.
                """
                rospy.logdebug("Distance to board: %f", xA)
                camera_y_mx = xA*np.arctan(self.camera_fov_h/2)
                camera_y_mn = -1*xA*np.arctan(self.camera_fov_h/2)
                camera_z_mx = xA*np.arctan(self.camera_fov_v/2)
                camera_z_mn = -1*xA*np.arctan(self.camera_fov_v/2)
                valve_y = (1-self.v_c[0]/self.camera_pix_h)*(camera_y_mx-
                    camera_y_mn)+camera_y_mn
                valve_z = (1-self.v_c[1]/self.camera_pix_v)*(camera_z_mx-
                    camera_z_mn)+camera_z_mn
                if self.tftree.frameExists("/base_laser") and self.tftree.frameExists("/camera"):
                    t = self.tftree.getLatestCommonTime("/base_laser",
                        "/camera")
                    posi, quat = self.tftree.lookupTransform("/base_laser", 
                        "/camera", t)
                    rospy.logdebug("TF Position from base_link to camera:")
                    rospy.logdebug(posi)
                    rospy.logdebug("TF Quaternion from base_link to camera:")
                    rospy.logdebug(quat)
                tf_x = posi[0]
                tf_y = posi[1]
                tf_z = posi[2]
                valve = np.array([xA, valve_y, valve_z],dtype=np.float32)
                rospy.loginfo("Valve in local coord. (x,y,z): (%f,%f,%f)",
                    valve[0], valve[1], valve[2])
                valve = valve+[tf_x,tf_y,tf_z]
                rospy.loginfo("Valve in global coord. (x,y,z): (%f,%f,%f)",
                    valve[0], valve[1], valve[2])
                rospy.set_param('valve2',[float(valve[0]), float(valve[1]),
                    float(valve[2])])
                # Set the position of the end effector with respect to the base
                rospy.set_param('ee_position',[float(tf_x), float(tf_y),
                    float(tf_z)])
                rospy.set_param('ugv_position',
                    [self.x0,self.y0,0,self.X0,self.Y0,self.Z0,self.W0])
                rospy.set_param('smach_state','valvepos')
                rospy.signal_shutdown('Ending node.')

            if self.fake_smach == 1:
                """This node centers the UGV on the valve. Note, currently
                we are bypassing this node because while holding the wrench
                we can't see the valve at the distance we specify. Once UGV
                is centered on valve, this node moves the UGV forward until
                it is a fixed distance from the board.
                """
                rospy.sleep(0.1)
                self.ct_move = 0
                wait_for_finish(100)
                valve = self.v_c[0]
                vw_c = valve #(valve+wrenc)/2
                vw_t = 800
                vw_off = (vw_c-vw_t)
                update_rot()

                # Check if we are centered on the valve
                if abs(vw_off) <= 500:
                    rospy.loginfo("UGV is centered on the valve.")

                    # Calculate the object location in local coordinate system
                    x_loc = ((xmx-xmn)/2)+xmn
                    y_loc = ((ymx-ymn)/2)+ymn
                    rospy.loginfo("Object in local coord (x,y): (%f,%f)",
                        x_loc, y_loc)
                    obj_loc = np.array([[x_loc],[y_loc]])
                    po = 0.8 # Distance off board we want to be 
                    back_it_up(0.25,(x_loc-po))
                    rospy.sleep(1)
                    self.fake_smach = 2
                else:
                    back_it_up(-0.25,1)

                    if vw_off < 0:
                        di = -0.1
                    else:
                        di = 0.1
                    self.goal.target_pose.pose = Pose(Point(
                        self.x0+di*np.sin(self.yaw),
                        self.y0-di*np.cos(self.yaw),0), 
                        Quaternion(self.X0,self.Y0,self.Z0,self.W0))
                    self.goal.target_pose.header.frame_id = 'odom'
                    self.move_base.send_goal(self.goal)
                    rospy.loginfo("UGV is not centered on valve.")
                    rospy.loginfo("Backing up and moving more centered.")
                    wait_for_finish(100)
                    rospy.sleep(self.rest_time*10)

            if self.fake_smach == 0:
                """This node moves to the initial estimate of valve location.
                """
                update_rot()
                valve = rospy.get_param('valve')
                ugv_pos = rospy.get_param('ugv_position')
                rospy.loginfo(valve)
                rospy.loginfo(ugv_pos)
                val_loc = [valve[0]-1,valve[1]]
                val_glo = np.dot(self.R,val_loc)
                self.x_val_glo = val_glo[0]+ugv_pos[0]
                self.y_val_glo = val_glo[1]+ugv_pos[1]
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose = Pose(Point(
                    self.x_val_glo,self.y_val_glo,0),
                    Quaternion(ugv_pos[3],ugv_pos[4],ugv_pos[5],ugv_pos[6]))
                self.goal.target_pose.header.frame_id = 'odom'
                self.move_base.send_goal(self.goal)
                rospy.loginfo("Moving to initial estimate of valve location.")
                wait_for_finish(100)
                self.fake_smach = 1
if __name__ == '__main__':
    try:
        drive2valve()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("drive2valve killed.")

