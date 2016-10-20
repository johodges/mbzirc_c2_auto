#!/usr/bin/env python

""" orient.py - Version 1.0 2016-10-12

    General framework based on Patrick Goebel's nav_test.py
    Initial version based on ccam-navigation by Chris Mobley

    Define waypoint destinations for a robot to move autonomously within
    a map framework.

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

    def __init__(self):
	rospy.init_node('orient', anonymous=True) # Give the node a name
        rospy.on_shutdown(self.shutdown) # Set rospy to execute a shutdown function when exiting
        self.rest_time = rospy.get_param("~rest_time", 0.1) # Minimum pause at each location
        self.stalled_threshold = rospy.get_param("~stalled_threshold", 100) # Loops before stall
        self.old_bearing = 0
        self.ct3 = 0
        self.flag = 0
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        self.locations = dict()
        self.wpname = dict()
        
        rospack = rospkg.RosPack()
        f = open(rospack.get_path('mbzirc_c2_auto')+'/params/pre-defined-path.txt','r')
        ct2 = 0
        with f as openfileobject:
            first_line = f.readline()
            for line in openfileobject:
                nome = [x.strip() for x in line.split(',')]
                self.wpname[ct2] = nome[0]
                x = Decimal(nome[1]); y = Decimal(nome[2]); z = Decimal(nome[3])
                X = Decimal(nome[4]); Y = Decimal(nome[5]); Z = Decimal(nome[6])
                self.locations[self.wpname[ct2]] = Pose(Point(x,y,z), Quaternion(X,Y,Z,1))
                ct2 = ct2+1
        self.wp = -1
        self.ct = 0
        self.ct_wrench = 0

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        self.goal = MoveBaseGoal()
        rospy.sleep(self.rest_time)
        rospy.Subscriber("/bearing", numpy_msg(Floats), self.callback, queue_size=1)
        rospy.Subscriber("/move_base/feedback", MoveBaseFeedback, self.callback_feedback, queue_size=1)
        rospy.Subscriber("/wrench_centroids", numpy_msg(Floats), self.callback_wrench, queue_size=1)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def callback_feedback(self, data):
        self.feedback = data

    def callback_wrench(self, data):
        self.wrench = data.data

    def callback(self, bearing):
        if self.wp == -1:
            print "Setting initial state."
            self.goal.target_pose.header.frame_id = 'base_link'
            self.goal.target_pose.pose = Pose(Point(-0.5,0,0), Quaternion(0,0,0,1))
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            rospy.sleep(0.5)
            #if self.move_base.get_state() == 3:
            self.wp = self.wp+1
            #    self.ct = 0
            #else:
            #    self.move_base.cancel_goal()
            #    self.ct = self.ct+1
            #    rospy.sleep(1)
            self.state = self.move_base.get_state()
        else:
            if bearing.data[0] != self.old_bearing:
                self.ct3 = self.ct3+1
            else:
                self.ct3 = 0
            if self.flag == 0:
                xA = bearing.data[1]
                yA = bearing.data[2]
                xB = bearing.data[3]
                yB = bearing.data[4]
                xmn = bearing.data[5]
                xmx = bearing.data[6]
                ymn = bearing.data[7]
                ymx = bearing.data[8]
                x0 = self.feedback.feedback.base_position.pose.position.x
                y0 = self.feedback.feedback.base_position.pose.position.y
                z0 = self.feedback.feedback.base_position.pose.position.z
                X0 = self.feedback.feedback.base_position.pose.orientation.x
                Y0 = self.feedback.feedback.base_position.pose.orientation.y
                Z0 = self.feedback.feedback.base_position.pose.orientation.z
                W0 = self.feedback.feedback.base_position.pose.orientation.w
                self.ct = 0

            if self.flag == 0:
                if self.ct3 < 5:
                    self.goal.target_pose.pose = Pose(Point(0,0,0), Quaternion(0,0,-bearing.data[0],1))
                    self.flag = 0
                    self.goal.target_pose.header.frame_id = 'base_link'
                else:
                #self.goal.target_pose.pose = Pose(Point(abs(yA)+trans[0]+np.cos(rot[2]),abs(xA)+5+trans[1]+np.sin(rot[2]),0), Quaternion(0,0,0,1))
                    print "Let's circle around!"
                    euler = tf.transformations.euler_from_quaternion([X0,Y0,Z0,W0])
                    roll = euler[0]
                    pitch = euler[1]
                    yaw = euler[2]
                    x_loc = ((xmx-xmn)/2)+xmn
                    y_loc = ((ymx-ymn)/2)+ymn
                    #y_loc = ymn+xmn*np.cos(-Z0)+ymn*np.sin(-Z0)#(ymx-ymn)/2+ymn*np.sin(-Z0)+xmn*np.cos(-Z0)
                    print "Object in local coord and local sys:", x_loc, y_loc, Z0
                    rospy.sleep(1)
                    #x_obj_loc = x_loc*np.cos(2*Z0)+y_loc*np.sin(2*Z0)
                    #y_obj_loc = -x_loc*np.sin(2*Z0)+y_loc*np.cos(2*Z0)
                    obj_loc = np.array([[x_loc],[y_loc]])
                    #theta = -2*Z0 # works for surface 1
                    #theta = 2*Z0 # works for surface 2
                    #theta = +Z0-0.78 # works for surface 2
                    theta = yaw#-Z0-0.78
                    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
                    print "Rotation Matrix:", R
                    tar_loc = np.array([[x_loc+y_loc+(ymx-ymn)/2],[y_loc+x_loc+(ymx-ymn)/2]])
                    print "Target loc in local coord and local sys:", tar_loc
                    obj_glo = np.dot(R,obj_loc)
                    tar_glo = np.dot(R,tar_loc)
                    x_obj_loc = obj_glo[0]
                    y_obj_loc = obj_glo[1]  
                    x_tar_loc = tar_glo[0]
                    y_tar_loc = tar_glo[1]
                    #print "Object in global sys:", obj_glo
                    #x_obj_loc = x_loc*np.sin(-Z0)+y_loc*np.cos(-Z0)
                    #y_obj_loc = x_loc*np.cos(-Z0)+y_loc*np.sin(-Z0)
                    print "Object in local coord and global sys:", x_obj_loc, y_obj_loc
                    x_obj_glo = x_obj_loc+x0
                    y_obj_glo = y_obj_loc+y0
                    print "Object in global coord and global sys:", x_obj_glo, y_obj_glo
                    #x_off = (-y_obj_loc*np.cos(Z0)+x_obj_loc*np.sin(Z0))#3*np.sin(Z0+0.78)#x_obj_loc
                    #y_off = (-y_obj_loc*np.sin(Z0)+x_obj_loc*np.cos(Z0))#3*np.cos(Z0+0.78)#y_obj_loc
                    #if abs(x_off) < 2:
                    #    x_off = 2*x_off/abs(x_off)
                    #if abs(y_off) < 2:
                    #    y_off = 2*y_off/abs(y_off)
                    print "Target in local coord and global sys:", x_tar_loc, y_tar_loc
                    #x_tar_glo = x0+1.5*(x_obj_loc-y_obj_loc)
                    #y_tar_glo = y0+1.5*(x_obj_loc+y_obj_loc)
                    x_tar_glo = x0+x_tar_loc
                    y_tar_glo = y0+y_tar_loc
                    print "Target in global coord and global sys:", x_tar_glo, y_tar_glo
                    rospy.sleep(5)
                    #x_off = x_glo
                    #y_off = y_glo
                    #print "Current location in global frame:", x0, y0
                    #print "Goal location in global frame:", x0+x_off, y0+y_off
                    #rospy.sleep(5)
                    print "Rotating to make path better."
                    loc = Pose(Point(x_tar_glo,y_tar_glo,0), Quaternion(0,0,yaw-0.78,1))
                    self.goal.target_pose.pose = Pose(Point(0,0,0),Quaternion(0,0,0.75,1))
                    self.goal.target_pose.header.frame_id = 'base_link'
                    self.move_base.send_goal(self.goal)
                    rospy.sleep(1)
                    self.ct_move = 0
                    while self.move_base.get_state() != 3:
                        if self.ct_move > 100:
                            self.move_base.send_goal(self.goal)
                            self.ct_move = 0
                        self.ct_move = self.ct_move + 1
                        rospy.sleep(0.1)
                    print "Done rotating!"
                    print "Moving around the box a bit to give clearance."
                    self.goal.target_pose.pose = Pose(Point(2,0,0),Quaternion(0,0,0,1))
                    self.move_base.send_goal(self.goal)
                    rospy.sleep(1)
                    self.ct_move = 0
                    while self.move_base.get_state() != 3:
                        if self.ct_move > 100:
                            self.move_base.send_goal(self.goal)
                            self.ct_move = 0
                        self.ct_move = self.ct_move + 1
                        rospy.sleep(0.1)
                    print "Done getting clearance."
                    print "Goal location in global frame:", x_tar_glo, y_tar_glo
                    print abs(xmx-xmn)+xmn, abs(ymx-ymn)+ymn+xmn
                    self.goal.target_pose.pose = loc
                    self.goal.target_pose.header.frame_id = 'odom'
                    self.move_base.send_goal(self.goal)
                    self.ct3 = 0
                    rospy.sleep(1)
                    self.ct_move = 0
                    while self.move_base.get_state() != 3:
                        if self.ct_move > 100:
                            self.move_base.send_goal(self.goal)
                            self.ct_move = 0
                        self.ct_move = self.ct_move + 1
                        rospy.sleep(0.1)
                self.goal.target_pose.header.stamp = rospy.Time.now()
            if self.flag == 0:
                self.move_base.send_goal(self.goal)
            else:
                #print self.feedback
                if self.move_base.get_state() == 3:
                    self.flag = 0
            #self.ct = 0
            self.old_bearing = bearing.data[0]
            rospy.sleep(1)
            state = self.move_base.get_state()
            if self.ct3 > 4:
                self.flag = 1
                rospy.sleep(5)
            if self.ct > 100:
                self.move_base.send_goal(self.goal)
                self.ct = 0
            else:
                self.ct = self.ct+1
            print np.shape(self.wrench)[0]
            if np.shape(self.wrench)[0] > 6:
                print "We found wrenches!"
                self.ct_wrench = self.ct_wrench+1
                self.ct3 = 0
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

