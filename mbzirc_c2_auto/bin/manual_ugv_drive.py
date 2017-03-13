#!/usr/bin/env python

"""manual_ugv_drive.py - Version 1.0 2016-12-27
Author: Alan Lattimer

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
from sensor_msgs.msg import Joy


class manual_orient():
    """Use keyboard to manually operate the UGV

    Reads the /keys topic to get keystrokes from stdin and then moves the UGV as:

      Q - Quit   W - Forward
       A - Left  S - Stop  D - Left
        Z - Auto  X - Backward

    """
    def __init__(self):
        # Name the node
        rospy.init_node('manual_ugv_drive', anonymous=True)

        # Log the instructions
        rospy.loginfo('Beginning manual operation of the UGV.')
        rospy.loginfo('  Operate the UGV by pressing the following keys:')
        rospy.loginfo('    w - move forward')
        rospy.loginfo('    x - move backward')
        rospy.loginfo('    a - turn left')
        rospy.loginfo('    d - turn right')
        rospy.loginfo('    s - stop')
        rospy.loginfo('    q - quit')
        rospy.loginfo('    z - return to autonomous operation')

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown_manops)

        # Start the key_puvlisher node
        self.get_keys = subprocess.Popen("rosrun mbzirc_c2_auto key_publisher.py", shell=True)

        # Subscribe to the keys topic
        self.key_sub = rospy.Subscriber('keys',String,self.key_cb)
        self.joy_sub = rospy.Subscriber('joy',Joy,self.joy_cb)

    def shutdown_manops(exit_str):
        """Kill the key_publisher node
        """
        rospy.logdebug('Attempting to kill keyboard_driver')
        rosnode.kill_nodes(['key_publisher'])

    def key_cb(self, data):
        """Callback used when a key is pressed and published
        """
        # Get the current keystroke and make it lowercase for later comparisons
        self.curr_key = data.data.lower()

        if self.curr_key == 'q':
            rospy.set_param('smach_state','stopManOps')
            rospy.sleep(1)
            rospy.signal_shutdown('Ending node. Unable to locate desired object.')
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

    def joy_cb(self, msg):
        if msg.buttons[12] == 1:
            rospy.loginfo("Move up")
        elif msg.buttons[13] == 1:
            rospy.loginfo("Move right")
        elif msg.buttons[14] == 1:
            rospy.loginfo("Move down")
        elif msg.buttons[15] == 1:
            rospy.loginfo("Move left")
        elif msg.buttons[10] == 1:
            rospy.signal_shutdown('Completed navigating')

if __name__ == '__main__':
    try:
        manual_orient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Manual UGV operation finished.")
