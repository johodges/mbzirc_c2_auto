#!/usr/bin/env python

"""key_pulisher.py - Version 1.0 2016-12-27
Author: Alan Lattimer

This reads keystrokes from the stdin stream and publishes them to a topic for
use in moving the robot

Publishers:
    /keys : keystrokes from stdin stream

Acknowledgments:
    This code was based on Example 8.1 of 'Programming Robots with ROS', M. Quigley,
    B. Gerkey, and W. D. Smart, 2015

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
import rosnode
import subprocess
from std_msgs.msg import String


class manual_orient():
    def __init__(self):
        # Name this node, it must be unique
        rospy.init_node('manual_orient', anonymous=True)

        rospy.loginfo('Beginning manual operation of the UGV.')
        rospy.loginfo('  Operate the UGV by pressing the following keys:')
        rospy.loginfo('    w - move forward')
        rospy.loginfo('    x - move backward')
        rospy.loginfo('    a - turn left')
        rospy.loginfo('    d - turn right')
        rospy.loginfo('    s - stop')
        rospy.loginfo('    q - quit')
        rospy.loginfo('    z - return to autonomous operation')

        # Enable shutdown in rospy
        rospy.on_shutdown(self.shutdown_manops) # Set rospy to execute a shutdown function when exiting

        self.get_keys = subprocess.Popen("rosrun mbzirc_c2_auto key_publisher.py", shell=True)

        self.key_sub = rospy.Subscriber('keys',String,self.callback)

    def shutdown_manops(exit_str):
        print exit_str
        rospy.loginfo('Attempting to kill keyboard_driver')
        rosnode.kill_nodes(['keyboard_driver'])

    # callback_wrench is used to store the wrench topic into the class to be
    # referenced by the other callback routines.
    def callback(self, data):
        self.curr_key = data.data.lower()

        if self.curr_key == 'q':
            rospy.set_param('smach_state','noWrenches')
            rospy.sleep(1)
            rospy.signal_shutdown('Ending node. Unable to find wrenches.')
        elif self.curr_key == 'z':
            rospy.set_param('smach_state','backToAuto')
            rospy.sleep(1)
            rospy.signal_shutdown('Ending node. Returning to autonomous mode.')
        elif self.curr_key == 'w':
            rospy.loginfo('UGV moving forward.')
        elif self.curr_key == 'x':
            rospy.loginfo('UGV moving backward.')
        elif self.curr_key == 'a':
            rospy.loginfo('UGV turning left.')
        elif self.curr_key == 'd':
            rospy.loginfo('UGV turning right.')
        elif self.curr_key == 's':
            rospy.loginfo('UGV stopping.')

if __name__ == '__main__':
    try:
        manual_orient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Manual UGV operation finished.")
