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
        self.stalled_threshold = rospy.get_param("~stalled_threshold", 1000) # Loops before stall

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

        # Set up ROS subscriber callback routines
        rospy.Subscriber("/bearing", numpy_msg(Floats), self.callback, queue_size=1)
        rospy.Subscriber("/move_base/feedback", MoveBaseFeedback, self.callback_feedback, queue_size=1)
        rospy.Subscriber("/wrench_centroids", numpy_msg(Floats), self.callback_wrench, queue_size=1)
        rospy.Subscriber("/wrench_center", numpy_msg(Floats), self.callback_w_c, queue_size=1)
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

    # callback_wrench is used to store the wrench topic into the class to be
    # referenced by the other callback routines.
    def callback_wrench(self, data):
        self.wrench = data.data

    # callback_w_c is used to store the wrench center topic into the class to be
    # referenced by the other callback routines.
    def callback_w_c(self, data):
        self.w_c = data.data

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
        def wait_for_finish():
            self.ct_move = 0
            rospy.sleep(1)
            while self.move_base.get_state() != 3:
                if self.ct_move > self.stalled_threshold:
                    back_it_up(-0.1,0.02)
                    rospy.sleep(0.5)
                    self.move_base.send_goal(self.goal)
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
            twi_pub = rospyPublisher = rospy.Publisher("/joy_teleop/cmd_vel", Twist, queue_size=10)
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
            twi_pub = rospyPublisher = rospy.Publisher("/joy_teleop/cmd_vel", Twist, queue_size=10)
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
            print "Setting initial state."
            rospy.loginfo("Setting initial state.")
            self.goal.target_pose.header.frame_id = 'base_link'
            self.goal.target_pose.pose = Pose(Point(-0.5,0,0), Quaternion(0,0,0,1))
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            rospy.sleep(0.5)
            self.wp = self.wp+1
            self.state = self.move_base.get_state()
        else:
            update_rot()
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
                xC = bearing.data[9]
                yC = bearing.data[10]

                # We check to see if we are bouncing between two angle increments, if
                # yes increment counter, otherwise set to 0.
                if self.theta != self.old_bearing:
                    self.ct3 = self.ct3+1
                else:
                    self.ct3 = 0

                # Reset the robot stuck counter
                self.ct = 0

                # Move the husky towards the median of the object
                err_dist = pow((xC*xC+yC*yC),0.5)
                q2 = tf.transformations.quaternion_from_euler(0,0,bearing.data[0])
                err_ang = pow(q2[0]*q2[0]+q2[1]*q2[1]+q2[2]*q2[2]+(q2[3]-1)*(q2[3]-1),0.5)
                print err_dist, err_ang
                rospy.loginfo('Error is:'
                              + str(err_dist) + 'meters '
                              + str(err_ang) + 'degrees')

                # Check if the error in distance and angle is below the threshold
                if err_dist > 0.4 or err_ang > 0.07:
                    tar_in_global([xC,yC+self.off])
                    print self.goal
                    q = tf.transformations.quaternion_from_euler(0,0,self.theta+bearing.data[0])
                    self.goal.target_pose.pose = Pose(Point(self.x_tar_glo,self.y_tar_glo,0), Quaternion(q[0],q[1],q[2],q[3]))
                    self.goal.target_pose.header.frame_id = 'odom'
                    self.flag = 0
                    self.old_bearing = self.theta
                    # Back up a bit to help the husky get to the target position
                    print "We should be backing up..."
                    back_it_up(-0.25,0.5)
                    print "We should be done backing up..."
                    self.move_base.send_goal(self.goal)
                    wait_for_finish()
                else:
                    # A flag of 1 means we should check for wrenches
                    self.flag = 1
            # A flag of 1 means we should check for wrenches
            if self.flag == 1:
                # Check if we see wrenches
                if np.shape(self.wrench)[0] > 6:
                    print "We found wrenches!"
                    rospy.loginfo("We found wrenches!")
                    self.ct_wrench = self.ct_wrench+1
                    self.ct3 = 0
                    # Make sure we saw wrenches 5 times through the loop (reduce false positives)
                    if self.ct_wrench > 5:
                        print "We are confident these are the wrenches we are looking for!"
                        rospy.sleep(1)
                        self.flag = 3
                        #rospy.signal_shutdown('Ending node.')
                else:
                    # If there are no wrenches, move to new location around box
                    self.ct_wrench = 0
                    self.flag = 2

            # A flag of 2 means we should move around the box. First we check if we can see
            # the far left end of the box. If we cannot, move along the box to the left 2m.
            # If we can, rotate 90 degrees around the box.
            if self.flag == 2:
                    # Determine bounds of camera FOV
                    camera_y_mx = xA*np.arctan(self.camera_fov_h/2)
                    camera_y_mn = -1*xA*np.arctan(self.camera_fov_h/2)
                    camera_z_mx = xA*np.arctan(self.camera_fov_v/2)
                    camera_z_mn = -1*xA*np.arctan(self.camera_fov_v/2)
                    print "Camera FOV: ", camera_y_mn, camera_y_mx
                    print "Box positions: ", ymn, ymx
                    update_rot()

                    # Check if the left end of the box is visible by the camera
                    if camera_y_mx > ymx:
                        self.off = 0
                        print "Let's circle around!"
                        print "Rotation Matrix:", self.R

                        # Calculate the object location in local coordinate system
                        x_loc = ((xmx-xmn)/2)+xmn
                        y_loc = ((ymx-ymn)/2)+ymn
                        #print "Object in local coord and local sys:", x_loc, y_loc, self.Z0
                        obj_loc = np.array([[x_loc],[y_loc]])
                        #print "ymx, ymn:", ymx, ymn

                        # Define the target location in the local coordinate system
                        tar_loc = np.array([[xmn+0.5],[ymx+3]])
                        print "Target loc in local coord and local sys:", tar_loc

                        # Convert local object and target locations to global coordinate system
                        obj_glo = np.dot(self.R,obj_loc)
                        self.x_obj_glo = obj_glo[0]+self.x0
                        self.y_obj_glo = obj_glo[1]+self.y0
                        tar_in_global(tar_loc)
                        print "Current location in global coord and global sys:", self.x0, self.y0
                        print "Object in global coord and global sys:", self.x_obj_glo, self.y_obj_glo
                        print "Target in global coord and global sys:", self.x_tar_glo, self.y_tar_glo

                        # Pause for a few seconds to allow user to cancel if needed
                        rospy.sleep(2)

                        # The path planner likes to try and run into the object. We force the
                        # robot to move in a specific direction initially to mitigate this.
                        print "Rotating to make path better."
                        q = tf.transformations.quaternion_from_euler(0,0,self.yaw-1.57)
                        loc = Pose(Point(self.x_tar_glo,self.y_tar_glo,0), Quaternion(q[0],q[1],q[2],q[3]))
                        rot_cmd(0.5,1.57)
                        print "Done rotating!"

                        # Move 2 meters along the box to help with the path planning
                        print "Moving around the box a bit to give clearance."
                        back_it_up(0.25,2)
                        print "Done getting clearance."
                        rospy.sleep(1)
                        # Move to target location
                        print "Goal location in global frame:", self.x_tar_glo, self.y_tar_glo

                        self.goal.target_pose.pose = loc
                        self.goal.target_pose.header.frame_id = 'odom'
                        self.move_base.send_goal(self.goal)
                        rospy.sleep(1)
                        self.ct3 = 0
                        self.flag = 1
                        wait_for_finish()
                        print "I should be at the target location!"
                        rospy.sleep(1)

                        # Move back to the positioning/normalization state
                        self.flag = 0
                    else:
                        # If we could not see the end of the box, offset along the box and
                        # Move back to the positioning/normalization state
                        self.off = self.off + 2
                        self.flag = 0
                    self.goal.target_pose.header.stamp = rospy.Time.now()

            # A flag of 4 means it is time to output the wrench and valve location to ROS
            # This is placed before flag = 3 so that an updated bearing topic will be obtained.
            if self.flag == 4:
                xA = bearing.data[1]
                yA = bearing.data[2]
                xB = bearing.data[3]
                yB = bearing.data[4]
                xmn = bearing.data[5]
                xmx = bearing.data[6]
                ymn = bearing.data[7]
                ymx = bearing.data[8]
                camera_y_mx = xA*np.arctan(self.camera_fov_h/2)
                camera_y_mn = -1*xA*np.arctan(self.camera_fov_h/2)
                camera_z_mx = xA*np.arctan(self.camera_fov_v/2)
                camera_z_mn = -1*xA*np.arctan(self.camera_fov_v/2)
                print self.v_c
                print self.w_c
                valve_y = (self.v_c[1]-0)/(2160-0)*(camera_y_mx-camera_y_mn)+camera_y_mn
                valve_z = (self.v_c[0]-0)/(3840-0)*(camera_z_mx-camera_z_mn)+camera_z_mn
                wrenc_y = (self.w_c[1]-0)/(2160-0)*(camera_y_mx-camera_y_mn)+camera_y_mn
                wrenc_z = (self.w_c[0]-0)/(3840-0)*(camera_z_mx-camera_z_mn)+camera_z_mn
                valve = np.array([xA, valve_y, valve_z],dtype=np.float32)
                wrench = np.array([xA, wrenc_y, wrenc_z],dtype=np.float32)
                wpub = rospy.Publisher('/wrench_mm', numpy_msg(Floats), queue_size=5)
                wpub.publish(wrench)
                vpub = rospy.Publisher('/valve_mm', numpy_msg(Floats), queue_size=5)
                vpub.publish(valve)
                print xmn, xmx, ymn, ymx
                print "Wrench location (x,y,z): ", wrench
                print "Valve location (x,y,z): ", valve

                # Store valve and wrench (x,y,z) location as ros parameters
                rospy.set_param('valve',[float(valve[0]), float(valve[1]), float(valve[2])])
                rospy.set_param('wrench',[float(wrench[0]), float(wrench[1]), float(wrench[2])])
                rospy.sleep(2)
                rospy.signal_shutdown('Ending node.')

            # A flag of 3 denotes centering between the valve and wrenches
            if self.flag == 3:
                rospy.sleep(1)
                self.ct_move = 0
                wait_for_finish()
                valve = self.v_c[0]
                wrenc = self.w_c[0]
                vw_c = (valve+wrenc)/2
                vw_t = 1920
                vw_off = (vw_c-vw_t)
                print "Target Center: ", vw_t, "Current Center: ", vw_c
                update_rot()

                # Check if we are centered between valve and wrenches
                if abs(vw_off) <= 100:
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
                    self.flag = 4
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
                    wait_for_finish()


if __name__ == '__main__':
    try:
        orient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("orient finished.")

