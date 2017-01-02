#!/usr/bin/env python

"""manual_arm_move.py - Version 1.0 2016-12-27
Author: Alan Lattimer

Reads in the keys topic an moves the end effector of the arm based on the
keys entered.

Subscribers:
    /keys : keystrokes from stdin stream

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


class manual_arm_move():
    def __init__(self):
        # Name the node
        rospy.init_node('manual_arm_move', anonymous=True)

        rospy.loginfo('Beginning manual operation of the arm.')
        rospy.loginfo('  Operate the arm by pressing the following keys:')
        rospy.loginfo('    k - move up')
        rospy.loginfo('    j - move down')
        rospy.loginfo('    h - move left')
        rospy.loginfo('    l - move right')
        rospy.loginfo('    u - move forward')
        rospy.loginfo('    m - move backward')
        rospy.loginfo('    a - rotate gripper counter-clockwise')
        rospy.loginfo('    d - rotate gripper clockwise')
        rospy.loginfo('    s - stop')
        rospy.loginfo('    q - quit')
        rospy.loginfo('    z - return to autonomous operation')

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown_manops)

        self.get_keys = subprocess.Popen("rosrun mbzirc_c2_auto key_publisher.py", shell=True)

        self.key_sub = rospy.Subscriber('keys',String,self.key_cb)

    def shutdown_manops(exit_str):
        """Kill the key_publisher node
        """
        rospy.logdebug('Attempting to kill keyboard_driver')
        rosnode.kill_nodes(['key_publisher'])

    def key_cb(self, data):
        """Callback used when a key is pressed and published
        """
        self.curr_key = data.data.lower()

        if self.curr_key == 'q':
            rospy.set_param('smach_state','endArmMove')
            rospy.sleep(1)
            rospy.signal_shutdown('Ending node. Stopping arm moves.')
        elif self.curr_key == 'z':
            rospy.set_param('smach_state','backToAuto')
            rospy.sleep(1)
            rospy.signal_shutdown('Ending node. Returning to autonomous mode.')
        elif self.curr_key == 'k':
            rospy.loginfo('Arm moving up.')
        elif self.curr_key == 'j':
            rospy.loginfo('Arm moving down.')
        elif self.curr_key == 'h':
            rospy.loginfo('Arm moving left.')
        elif self.curr_key == 'l':
            rospy.loginfo('Arm moving right.')
        elif self.curr_key == 'u':
            rospy.loginfo('Arm moving forward.')
        elif self.curr_key == 'm':
            rospy.loginfo('Arm moving backward.')
        elif self.curr_key == 'a':
            rospy.loginfo('Gripper turning counter-clockwise')
        elif self.curr_key == 'd':
            rospy.loginfo('Gripper turning clockwise')
        elif self.curr_key == 's':
            rospy.loginfo('Arm stopping.')

if __name__ == '__main__':
    try:
        manual_arm_move()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Manual Arm operation finished.")
