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


class init_smach():
    """Use keyboard to start SMACH

    Press the space bar to start SMACH

    """
    def __init__(self):
        # Name the node
        rospy.init_node('init_smach_ops', anonymous=True)

        # Log the instructions
        rospy.loginfo('Press the space bar to start SMACH')

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown_manops)

        # Start the key_puvlisher node
        self.get_keys = subprocess.Popen("rosrun mbzirc_c2_auto key_publisher.py", shell=True)

        # Subscribe to the keys topic
        self.key_sub = rospy.Subscriber('keys',String,self.key_cb)

    def shutdown_manops(exit_str):
        """Kill the key_publisher node
        """
        rospy.logdebug('Attempting to kill keyboard_driver')
        rosnode.kill_nodes(['key_publisher'])
        rospy.sleep(0.1)

    def key_cb(self, data):
        """Callback used when a key is pressed and published
        """
        # Get the current keystroke and make it lowercase for later comparisons
        self.curr_key = data.data.lower()

        if self.curr_key == ' ':
            rospy.set_param('smach_state','startSMACHing')
            rospy.sleep(1)
            rospy.signal_shutdown('Ending node to continue SMACH')

if __name__ == '__main__':
    try:
        init_smach()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH initialized")
