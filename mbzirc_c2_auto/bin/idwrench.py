#!/usr/bin/env python

""" idwrench.py - Version 1.0 2016-10-12

    This software chooses the left most wrench in an RGB image and outputs an
    estimate of its 3D location in space relative to the camera [x,y,z]
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

class idwrench():
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('idwrench', anonymous=True)

        # Enable shutdown in rospy (This is important so we cancel any move_base goals
        # when the node is killed)
        rospy.on_shutdown(self.shutdown) # Set rospy to execute a shutdown function when exiting

        self.flag = 0

        # Store camera parameters
        self.camera_fov_h = 1.5708
        self.camera_fov_v = 1.5708
        self.camera_pix_h = 1920
        self.camera_pix_v = 1080

        # Set up ROS subscriber callback routines+
        rospy.Subscriber("/wrench_centroids", numpy_msg(Floats), self.callback_wrench, queue_size=1)

    def shutdown(self):
        rospy.sleep(0.0001)

    # callback_wrench is used to store the wrench topic into the class to be
    # referenced by the other callback routines.
    def callback_wrench(self, data):

        if np.shape(data.data)[0] > 6:
            tmp = np.reshape(data.data,(np.size(data.data)/2,2))
            self.wrench = tmp[tmp[:,0].argsort()]
            self.left_wrench = self.wrench[0,:]
            wrench = rospy.get_param('wrench')
            xA = wrench[0]
            print "Left wrench in pixels: ", self.left_wrench
            camera_y_mx = xA*np.tan(self.camera_fov_h/2)
            camera_y_mn = -1*xA*np.tan(self.camera_fov_h/2)
            camera_z_mx = xA*np.tan(self.camera_fov_v/2)
            camera_z_mn = -1*xA*np.tan(self.camera_fov_v/2)
            print "Camera ymn/ymx: ", camera_y_mn, camera_y_mx
            wrenc_y = (1-self.left_wrench[0]/1920)*(camera_y_mx-camera_y_mn)+camera_y_mn
            wrenc_z = (1-self.left_wrench[1]/1080)*(camera_z_mx-camera_z_mn)+camera_z_mn
            self.wrench_id = np.array([xA, wrenc_y, wrenc_z],dtype=np.float32)
            print "Left wrench in m: ", self.wrench_id
            rospy.set_param('wrench_ID',[float(self.wrench_id[0]), float(self.wrench_id[1]), float(self.wrench_id[2])])
            rospy.set_param('smach_state','wrenchFound')
        else:
            rospy.set_param('smach_state','wrenchNotFound')

        rospy.signal_shutdown('Ending node.')

if __name__ == '__main__':


    rospy.set_param('smach_state','armTest')

    try:
        idwrench()
        print "I'm done!"
#        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("idwrench finished.")

