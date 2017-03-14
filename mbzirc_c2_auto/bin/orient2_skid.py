#!/usr/bin/env python

""" orient2.py - Version 1.0 2016-10-12
Author: Jonathan Hodges

This software uses a LIDAR scan to move around a box looking for wrenches.

Subscribers:
    /bearing: array containing [angle,x_med,xmn,xmx,ymn,ymx,target_x,target_y]
    /move_base/feedback: Topic containing current position and orientation of
        UGV from move_base server
    /valve: Topic containing estimated center of valve (I don't think this is
        currently in use)
    /wrench_center: Topic containing center of all wrenches
    /wrench_centroids: Topic containing center of wrenches in pixels
    /tf: TF tree

Publishers:
    /wrench_mm: wrench location in global coordinates
    /valve_mm: valve location in global coordinates
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
    wrenches_found: feedback to wrench_detect wrenches have been found

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
import rospkg
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
from decimal import *
import tf
import math

class orient():
    """This class drives the husky around the board looking for wrenches.



    Attributes:
        big_board_offset: offset from center of the board moving left to ensure
            the whole board has been checked.
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
        v_c: valve center
        vpub: Valve position estimate
        w_c: Wrench centroid from wrench_detect
        wp: flag to get a goal at the start
        wpub: Wrench position estimate
        wrench: Position of all wrenches detected in wrench_detect
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
            of the UGV from move_base server
        /valve: Topic containing estimated center of valve (I don't think this
            is currently in use)
        /wrench_center: Topic containing center of all wrenches
        /wrench_centroids: Topic containing center of wrenches in pixels
        /tf: TF tree

    Publishers:
        /wrench_mm: wrench location in global coordinates
        /valve_mm: valve location in global coordinates
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
        wrenches_found: feedback to wrench_detect wrenches have been found

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
        rospy.init_node('orient', anonymous=True)
        rospy.on_shutdown(self.shutdown) # Enable shutdown in rospy

        #Initialize parameters
        self.rest_time = 0.1            # Minimum pause at each location
        self.stalled_threshold = 5000    # Loops before stall
        self.fake_smach = 0             # Flag to transition to diff actions
        self.wp = -1                    # Set to -1 to get goal at start
        self.wrench_counter = 0         # Consecutive loops wrenches seen
        self.big_board_offset = 0       # Offset for a large board
        
        # Hardware Parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base",
            MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        self.goal = MoveBaseGoal()

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting orientation.")

        # Establish publishers and subscribers
        self.twi_pub = rospy.Publisher("/cmd_vel", Twist,
            queue_size=5)
        self.wpub = rospy.Publisher('/wrench_mm', numpy_msg(Floats),
            queue_size=5)
        self.vpub = rospy.Publisher('/valve_mm', numpy_msg(Floats),
            queue_size=5)
        self.tftree = tf.TransformListener() # Set up tf listener
        rospy.Subscriber("/bearing", numpy_msg(Floats), self.callback,
            queue_size=1)
        rospy.Subscriber("/valve", numpy_msg(Floats), self.callback_v_c,
            queue_size=1)
        rospy.Subscriber("/wrench_centroids", numpy_msg(Floats),
            self.callback_wrench, queue_size=1)
        rospy.Subscriber("/wrench_center", numpy_msg(Floats),
            self.callback_w_c, queue_size=1)
        rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        rospy.Subscriber('/odometry/filtered',Odometry, self.callback_odom)


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

    def callback_odom(self, data):
        self.odom = data.pose.pose

    def callback_wrench(self, data):
        """This callback is used to store the all the detected wrenches into
        the class to be referenced by the other callback routines.
        """
        self.wrench = data.data

    def callback_w_c(self, data):
        """This callback is used to store the wrench cent. topic into the class
        to be referenced by the other callback routines.
        """
        self.w_c = data.data

    def callback_v_c(self, data):
        """This callback is used to store the valve center topic into the class
        to be referenced by the other callback routines.
        """
        self.v_c = data.data

    def callback_imu(self, data):
        """This callback is used to store the imu topic into the class
        to be referenced by the other callback routines.
        """
        gyro = data.orientation
        euler = tf.transformations.euler_from_quaternion([gyro.x,gyro.y,gyro.z,gyro.w])
        self.imu_yaw = euler[2]

    def callback(self, bearing):
        """This callback drives around the board looking for wrenches. Once
        found, it centers on the wrenches and moves close to the board.

        The general process is:
            1. Move to wrench detection position from /bearing topic
            2. Check if wrenches are visible on the surface
            3. If no wrenches are visible, check if camera can see left edge of
                board. If no, move the husky left 2 meters and repeat step 1-2.
                If yes, move the husky clockwise around the panel.
            4. Repeat 1-3 until wrenches are identified.
            5. Kill this node

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
                    rospy.sleep(1)
                    back_it_up(-0.25,0.5)
                    rospy.sleep(0.5)
                    rot_cmd(1.0,0.25)
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

        #def rot_cmd(ve,dist_to_move):
            """This subroutine manually rotates the husky along the z-axis a
            fixed amount at a fixed velocity by bypassing the move_base package
            and sending a signal directly to the wheels.

            This subroutine is likely to be replaced by:
                file:
                    robot_mv_cmds
                    subroutine: move_UGV_vel(lv,av,dist_to_move)
            """
            #current_yaw = self.imu_yaw
            #target_yaw = current_yaw+dist_to_move
            #time_to_move = abs(dist_to_move/ve)
            #twist = Twist()
            #twist.angular.z = ve
            #self.ct_move = 0
            #while abs(target_yaw-self.imu_yaw) > 0.1:
            #    self.twi_pub.publish(twist)
            #    self.ct_move = self.ct_move+1
            #    print "rot error: ", abs(target_yaw-self.imu_yaw)
            #    rospy.sleep(self.rest_time)
            #return None

        def rot_cmd(ve,dist_to_move):
            update_rot()
            rospy.loginfo("Current position:")
            rospy.loginfo(self.yaw)
            rospy.loginfo("Target position:")
            rospy.loginfo(self.yaw+dist_to_move)
            q = tf.transformations.quaternion_from_euler(
                0,0,self.yaw+dist_to_move)
            self.goal.target_pose.pose = Pose(Point(self.x0,self.y0,0),
                Quaternion(q[0],q[1],q[2],q[3]))
            self.goal.target_pose.header.frame_id = 'odom'
            self.move_base.send_goal(self.goal)
            rospy.loginfo("Skid steer")
            wait_for_finish(self.stalled_threshold)
            rospy.sleep(5)

        # update_rot - this subroutine updates the feedback locations
        def update_rot():
            """This subroutine updates the feedback locations in the global
            reference frame from the /feedback topic.
            """
            self.x0 = self.odom.position.x
            self.y0 = self.odom.position.y
            self.z0 = self.odom.position.z
            self.X0 = self.odom.orientation.x
            self.Y0 = self.odom.orientation.y
            self.Z0 = self.odom.orientation.z
            self.W0 = self.odom.orientation.w
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
            """
        def skid_steer_move(target_pose,target_orient):
            update_rot()
            rospy.loginfo("Current position:")
            rospy.loginfo([self.x0, self.y0])
            rospy.loginfo("Target position:")
            rospy.loginfo(target_pose)
            mid_ang = np.arctan((target_pose[1]-self.y0)/(target_pose[0]-self.x0))
            euler = tf.transformations.euler_from_quaternion(target_orient)
            ang_to_rot = mid_ang-self.yaw
            rospy.loginfo("current angle")
            rospy.loginfo(self.yaw)
            rospy.loginfo("mid_ang")
            rospy.loginfo(mid_ang)
            rospy.loginfo("Starting Skid Steer #1.")
            rot_cmd(1.0*ang_to_rot/abs(ang_to_rot),ang_to_rot)
            rospy.sleep(10)
            q = tf.transformations.quaternion_from_euler(
                0,0,mid_ang)
            q = target_pose
            self.goal.target_pose.pose = Pose(Point(target_pose[0],target_pose[1],0),
                Quaternion(q[0],q[1],q[2],q[3]))
            self.goal.target_pose.header.frame_id = 'odom'
            rospy.loginfo("Starting drive to target pose.")
            self.move_base.send_goal(self.goal)
            wait_for_finish(self.stalled_threshold)
            rospy.sleep(10)
            update_rot()
            """
            """
            self.goal.target_pose.pose = Pose(Point(target_pose[0],target_pose[1],0),
                Quaternion(target_orient[0],target_orient[1],target_orient[2],target_orient[3]))
            self.goal.target_pose.header.frame_id = 'odom'
            rospy.loginfo("Starting Skid Steer #2.")
            self.move_base.send_goal(self.goal)
            wait_for_finish(self.stalled_threshold)
            rospy.sleep(20)
            """
            """
            return None
            """



        def skid_steer_move(target_pose,target_orient):
            update_rot()
            rospy.loginfo("Current position:")
            rospy.loginfo([self.x0, self.y0])
            rospy.loginfo("Target position:")
            rospy.loginfo(target_pose)
            mid_ang = np.arctan((target_pose[1]-self.y0)/(target_pose[0]-self.x0))
            q = tf.transformations.quaternion_from_euler(
                0,0,mid_ang)
            self.goal.target_pose.pose = Pose(Point(self.x0,self.y0,0),
                Quaternion(q[0],q[1],q[2],q[3]))
            self.goal.target_pose.header.frame_id = 'odom'
            self.move_base.send_goal(self.goal)
            rospy.loginfo("Skid steer 1")
            wait_for_finish(self.stalled_threshold)
            rospy.sleep(5)
            self.goal.target_pose.pose = Pose(Point(target_pose[0],target_pose[1],0),
                Quaternion(q[0],q[1],q[2],q[3]))
            self.goal.target_pose.header.frame_id = 'odom'
            self.move_base.send_goal(self.goal)
            rospy.loginfo("Starting drive to target pose.")
            wait_for_finish(self.stalled_threshold)
            rospy.sleep(5)
            q = target_orient
            self.goal.target_pose.pose = Pose(Point(target_pose[0],target_pose[1],0),
                Quaternion(q[0],q[1],q[2],q[3]))
            self.goal.target_pose.header.frame_id = 'odom'

            self.move_base.send_goal(self.goal)
            rospy.loginfo("Skid steer 2")
            wait_for_finish(self.stalled_threshold)
            rospy.sleep(5)
            update_rot()
            """
            self.goal.target_pose.pose = Pose(Point(target_pose[0],target_pose[1],0),
                Quaternion(target_orient[0],target_orient[1],target_orient[2],target_orient[3]))
            self.goal.target_pose.header.frame_id = 'odom'
            rospy.loginfo("Starting Skid Steer #2.")
            self.move_base.send_goal(self.goal)
            wait_for_finish(self.stalled_threshold)
            rospy.sleep(20)
            """
            return None


        if self.wp == -1:
            """We need some feedback from the move_base server to obtain our
            current location. Since the move_base server does not publish
            feedback without an active goal, we set an initial goal to get our
            position.

            Ideally this would be replaced by pulling the position of the UGV
            from a different feedback topic or the TF tree.
            """
            rospy.loginfo("Setting initial state.")
            self.goal.target_pose.header.frame_id = 'base_link'
            self.goal.target_pose.pose = Pose(Point(-0.5,0,0),
                Quaternion(0,0,0,1))
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            rospy.sleep(self.rest_time*10)
            self.wp = self.wp+1
            self.state = self.move_base.get_state()
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

            if self.fake_smach == 4:
                """This node outputs the wrench and valve location to ROS and
                kills this node.
                """
                camera_y_mx = xA*np.arctan(self.camera_fov_h/2)
                camera_y_mn = -1*xA*np.arctan(self.camera_fov_h/2)
                camera_z_mx = xA*np.arctan(self.camera_fov_v/2)
                camera_z_mn = -1*xA*np.arctan(self.camera_fov_v/2)

                rospy.logdebug("Camera (ymn,ymx): (%f,%f)", camera_y_mn,
                    camera_y_mx)
                rospy.logdebug("Camera (zmn,zmx): (%f,%f)", camera_z_mn,
                    camera_z_mx)
                rospy.logdebug("Valve pixel (c,r): (%f,%f)", self.v_c[0],
                    self.v_c[1])
                rospy.logdebug("Wrench pixel (c,r): (%f,%f)", self.w_c[0],
                    self.w_c[1])
                valve_y = (1-self.v_c[0]/self.camera_pix_h)*(camera_y_mx-
                    camera_y_mn)+camera_y_mn
                valve_z = (1-self.v_c[1]/self.camera_pix_v)*(camera_z_mx-
                    camera_z_mn)+camera_z_mn
                wrenc_y = (1-self.w_c[0]/self.camera_pix_h)*(camera_y_mx-
                    camera_y_mn)+camera_y_mn
                wrenc_z = (1-self.w_c[1]/self.camera_pix_v)*(camera_z_mx-
                    camera_z_mn)+camera_z_mn
                try:
                    lidar_to_use = rospy.get_param('lidar')
                except:
                    lidar_to_use = 'sick'
                if lidar_to_use == 'sick':
                    if self.tftree.frameExists("/base_link") and self.tftree.frameExists("/gripper_camera"):
                        t = self.tftree.getLatestCommonTime("/base_link",
                            "/gripper_camera")
                        posi, quat = self.tftree.lookupTransform("/base_link", 
                            "/gripper_camera", t)
                        rospy.logdebug("TF Position from base_link to camera:")
                        rospy.logdebug(posi)
                        rospy.logdebug("TF Quaternion from base_link to camera:")
                        rospy.logdebug(quat)
                if lidar_to_use == 'velodyne':
                    if self.tftree.frameExists("/base_link") and self.tftree.frameExists("/gripper_camera"):
                        t = self.tftree.getLatestCommonTime("/base_link",
                            "/gripper_camera")
                        posi, quat = self.tftree.lookupTransform("/base_link", 
                            "/gripper_camera", t)
                        rospy.logdebug("TF Position from base_link to camera:")
                        rospy.logdebug(posi)
                        rospy.logdebug("TF Quaternion from base_link to camera:")
                        rospy.logdebug(quat)
                tf_x = posi[0]
                tf_y = posi[1]
                tf_z = posi[2]
                valve = np.array([xA, valve_y, valve_z],dtype=np.float32)
                wrench = np.array([xA, wrenc_y, wrenc_z],dtype=np.float32)

                self.wpub.publish(wrench)
                self.vpub.publish(valve)

                wrench = wrench+[tf_x,tf_y,tf_z]
                valve = valve+[tf_x,tf_y,tf_z]

                rospy.loginfo("Wrench location (x,y,z): (%f,%f,%f)",
                    wrench[0], wrench[1], wrench[2])
                rospy.loginfo("Valve location (x,y,z): (%f,%f,%f)",
                    valve[0], valve[1], valve[2])

                # Store valve and wrench (x,y,z) location as ros parameters
                rospy.set_param('valve',[float(valve[0]), float(valve[1]),
                    float(valve[2])])
                rospy.set_param('wrench',[float(wrench[0]), float(wrench[1]),
                    float(wrench[2])])

                # Set the current position of the end effector with respect to
                # the base
                rospy.set_param('ee_position',[float(tf_x), float(tf_y),
                    float(tf_z)])
                rospy.set_param('stow_position',[float(tf_x), float(tf_y),
                    float(tf_z)])
                rospy.set_param('current_joint_state',[0,0,0,0,0,0])
                rospy.set_param('ugv_position',
                    [self.x0,self.y0,0,self.X0,self.Y0,self.Z0,self.W0])
                rospy.set_param('smach_state','oriented')

                rospy.signal_shutdown('Ending node.')

            # A flag of 3 denotes centering between the valve and wrenches
            if self.fake_smach == 3:
                """This node centers the UGV on the wrenches. Once the UGV
                is centered on the wrenches, this node moves the UGV forward
                until it is a fixed distance from the board.
                """
                rospy.set_param('wrenches_found',1)
                rospy.sleep(self.rest_time)
                self.ct_move = 0
                wait_for_finish(self.stalled_threshold)
                valve = self.v_c[0]
                wrenc = self.w_c[0]
                vw_c = wrenc #(valve+wrenc)/2
                vw_t = 800
                vw_off = (vw_c-vw_t)

                camera_y_mx = xA*np.arctan(self.camera_fov_h/2)
                camera_y_mn = -1*xA*np.arctan(self.camera_fov_h/2)
                camera_z_mx = xA*np.arctan(self.camera_fov_v/2)
                camera_z_mn = -1*xA*np.arctan(self.camera_fov_v/2)
                offset = -0.1
                wrenc_y = (1-self.w_c[0]/self.camera_pix_h)*(camera_y_mx-
                    camera_y_mn)+camera_y_mn
                wrenc_z = (1-self.w_c[1]/self.camera_pix_v)*(camera_z_mx-
                    camera_z_mn)+camera_z_mn
                rospy.loginfo("WRENCH_Y+OFFSET = %f", wrenc_y+offset)
                # Check if we are centered between valve and wrenches
                if abs(wrenc_y+offset) <= 0.15 and abs(bearing.data[0] < 0.1):
                    rospy.loginfo("UGV is centered on the wrenches.")

                    # Calculate the object location in local coordinate system
                    x_loc = ((xmx-xmn)/2)+xmn
                    y_loc = ((ymx-ymn)/2)+ymn
                    rospy.loginfo("Object in local coord (x,y): (%f,%f)",
                        x_loc, y_loc)
                    obj_loc = np.array([[x_loc],[y_loc]])
                    po = 0.8 # Distance off board we want to be 
                    back_it_up(0.25,(x_loc-po))
                    self.fake_smach = 4
                else:
                    q = tf.transformations.quaternion_from_euler(
                        0,0,self.theta+ang)
                    rospy.loginfo("UGV is not centered on wrenches.")
                    back_it_up(-0.25,1)
                    tar_glo = np.dot(self.R,[bearing.data[1]-2,wrenc_y+offset])
                    x_wre = tar_glo[0]+self.x0
                    y_wre = tar_glo[1]+self.y0
                    skid_steer_move([x_wre,y_wre],q)
                    #self.goal.target_pose.pose = Pose(Point(x_wre,y_wre,0),
                    #    Quaternion(q[0],q[1],q[2],q[3]))
                    #self.goal.target_pose.header.frame_id = 'odom'
                    #self.move_base.send_goal(self.goal)

                    rospy.loginfo("Backing up and moving more centered.")
                    wait_for_finish(self.stalled_threshold)
                    rospy.sleep(self.rest_time*10)

            if self.fake_smach == 2:
                """This node moves around the box. First we check to see if
                the camera can see the far left of the box. If we cannot, move
                along the box to the left 2m. If we can, rotating 90 degrees
                around the box.
                """

                # Determine bounds of camera FOV
                camera_y_mx = xA*np.tan(self.camera_fov_h/2)
                camera_y_mn = -1*xA*np.tan(self.camera_fov_h/2)
                camera_z_mx = xA*np.tan(self.camera_fov_v/2)
                camera_z_mn = -1*xA*np.tan(self.camera_fov_v/2)

                # Check if the left end of the box is visible by the camera
                if True: #camera_y_mx > ymx:
                    self.big_board_offset = 0
                    rospy.loginfo("No wrenches on this side. Circling around.")

                    # Calculate the object location in local coordinate system
                    x_loc = ((xmx-xmn)/2)+xmn
                    y_loc = ((ymx-ymn)/2)+ymn
                    rospy.logdebug("Object in local coord (x,y): (%f,%f)",
                        x_loc, y_loc)
                    obj_loc = np.array([[x_loc],[y_loc]])

                    # Define the target location in the local coordinate system
                    tar_loc = np.array([[xmn+1],[ymx+2]])
                    # originally 0.5 and 1
                    rospy.logdebug("Target in local coord (x,y): (%f,%f)",
                        tar_loc[0], tar_loc[1])

                    # Convert local object and target locations to global coord
                    obj_glo = np.dot(self.R,obj_loc)
                    self.x_obj_glo = obj_glo[0]+self.x0
                    self.y_obj_glo = obj_glo[1]+self.y0
                    tar_in_global(tar_loc)

                    rospy.loginfo("Target in global coord (x,y): (%f,%f)",
                        self.x_tar_glo, self.y_tar_glo)

                    # Pause to allow user to cancel if needed
                    rospy.sleep(self.rest_time)

                    # The path planner likes to try and run into the object. We
                    # force the robot to move in a specific direction initially
                    # to mitigate this.
                    #rospy.loginfo("Rotating to make path better.")
                    q = tf.transformations.quaternion_from_euler(
                        0,0,self.yaw-1.57)
                    #loc = Pose(Point(self.x_tar_glo,self.y_tar_glo,0),
                    #    Quaternion(q[0],q[1],q[2],q[3]))
                    #rot_cmd(0.5,1.57)
                    #rospy.loginfo("Done pre-rotating.")

                    # Move 2 meters along the box to help with the path plan
                    #rospy.loginfo("Moving forward to obtain clearance.")
                    #back_it_up(0.50,1)
                    #rospy.loginfo("Done getting clearance.")
                    #rospy.sleep(self.rest_time)
                    #rospy.loginfo("Rotating toward box (skid steer).")
                    #rot_cmd(-0.5,-1.57)
                    #rospy.loginfo("Done with skid steer.")
                    #rospy.sleep(self.rest_time)
                    rospy.logdebug("Move to target location.")
                    skid_steer_move([self.x_tar_glo,self.y_tar_glo],q)
                    #self.goal.target_pose.pose = loc
                    #self.goal.target_pose.header.frame_id = 'odom'
                    #self.move_base.send_goal(self.goal)
                    rospy.sleep(5.0)
                    self.fake_smach = 1
                    wait_for_finish(self.stalled_threshold)
                    rospy.loginfo("UGV at target location on next side.")
                    rospy.sleep(self.rest_time)

                    # Move back to the positioning/normalization state
                    self.fake_smach = 0
                else:
                    # If we could not see the end of the box, offset along the
                    # box and move back to the positioning/normalization state
                    rospy.loginfo("No wrenches were found.")
                    rospy.loginfo("Could not see left edge of box.")
                    rospy.loginfo("Moving to the left 2m")
                    self.big_board_offset = self.big_board_offset + 2
                    self.fake_smach = 0
                self.goal.target_pose.header.stamp = rospy.Time.now()

            if self.fake_smach == 1:
                """This node checks for wrenches once the UGV is positioned
                correctly.
                """
                # Check if we see wrenches
                if np.shape(self.wrench)[0] > 10:
                    
                    self.wrench_counter = self.wrench_counter+1
                    rospy.sleep(self.rest_time)
                    # Make sure we saw wrenches 5 times through the loop
                    if self.wrench_counter > 5:
                        rospy.loginfo("Wrenches were detected.")
                        rospy.sleep(self.rest_time*10)
                        self.fake_smach = 3
                else:
                    # If there are no wrenches, move to new location around box
                    self.wrench_counter = 0
                    self.fake_smach = 2

            # Check if we are currently trying to rotate around the panel
            if self.fake_smach == 0:
                """This node moves the UGV such that is in the correct position
                to detect wrenches.
                """

                # Move the husky towards the median of the object
                err_dist = pow((xC*xC+yC*yC),0.5)
                q2 = tf.transformations.quaternion_from_euler(0,0,ang)
                err_ang = pow(q2[0]*q2[0]+q2[1]*q2[1]+q2[2]*q2[2]
                    +(q2[3]-1)*(q2[3]-1),0.5)
                # print err_dist, err_ang
                rospy.logdebug('Error is:'
                               + str(err_dist) + 'meters '
                               + str(err_ang) + 'degrees')

                # Check if the error in dist and angle is below the threshold
                #if err_dist > 0.2 or err_ang > 0.07:
                if err_dist > 0.4 or err_ang > 0.2:
                    tar_in_global([xC,yC+self.big_board_offset])
                    # print self.goal
                    q = tf.transformations.quaternion_from_euler(
                        0,0,self.theta+ang)
                    self.goal.target_pose.pose = Pose(Point(self.x_tar_glo,
                        self.y_tar_glo,0), Quaternion(q[0],q[1],q[2],q[3]))
                    self.goal.target_pose.header.frame_id = 'odom'
                    self.fake_smach = 0
                    # Back up a bit to help the husky get to the target pos
                    back_it_up(-0.25,0.5)
                    skid_steer_move([self.x_tar_glo,self.y_tar_glo],q)
                    #self.move_base.send_goal(self.goal)
                    #wait_for_finish(self.stalled_threshold)
                else:
                    rospy.loginfo("UGV is in position, check for wrenches.")
                    rospy.sleep(self.rest_time*10)
                    self.fake_smach = 1

if __name__ == '__main__':
    try:
        orient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Orient killed.")

