""" grasp_wrench_states.py - Version 1.0 2016-11-10

    State machine classes for grasping a certain wrench from a peg board.

    Classes:
        MoveToReady -
        MoveToWrenchReady
        IDWrench
        MoveToWrench
        MoveToGrasp
        GraspWrench

    Alan Lattimer (alattimer at jensenhughes dot com)

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
import smach
import subprocess
from geometry_msgs.msg import Twist
from mbzirc_c2_auto.msg import kf_msg
from robot_mv_cmds import *

rospy.init_node('move_arm_once', anonymous=True) # Name this node

x = 0.37
y = 0.17974881827831268
z = -0.13981889188289642
moveArmTwist(x,y,z)

