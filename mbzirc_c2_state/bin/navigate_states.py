""" navigate_states.py - Version 1.0 2016-11-10

    State machine classes for finding and navigating to the board containing the
    wrenches and valve.

    Classes
        FindBoard

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


class FindBoard(smach.State):
    """Searches for and then navigates to the board

    Outcomes
    --------
        atBoard : at the board location

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atBoard',
                                       'skipNav'],
                             input_keys=['test_arm_in'])

    def execute(self, userdata):

        if userdata.test_arm_in is True:
            rospy.set_param('wrench',[1.50195926, -0.02785565, 0.40498503])
            rospy.set_param('valve',[1.50195926, 0.23329135, 0.22560422])
            rospy.set_param('ee_position',[0.4863007578206243,
                                           0.10914527097633821,
                                           0.6197410984973739])
            rospy.set_param('current_joint_state', [0, 0, 0, 0, 0, 0])
            return 'skipNav'

        else:
            rospy.loginfo('Searching for board')
            a = subprocess.Popen("rosrun mbzirc_c2_auto findbox.py", shell=True)
            b = subprocess.Popen("rosrun mbzirc_c2_auto autonomous.py", shell=True)

            b.wait()
            rospy.loginfo('Searching for board')
            a.kill()

            return 'atBoard'

