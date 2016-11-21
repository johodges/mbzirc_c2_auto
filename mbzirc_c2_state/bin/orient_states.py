""" orient_states.py - Version 1.0 2016-11-10

    State machine classes for orienting the Husky around the board and positioning
    for grasping the wrench and operating the valve.

    Classes
        Orient

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


class Orient(smach.State):
    """Orients the Husky to the correct face

    Orients around the board to the wrench face and orients the Husky
    normal to the wrench face.

    Outcomes
    --------
      oriented : Husky oriented to grasp wrench and turn valve

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['oriented'])

    def execute(self, userdata):
        rospy.loginfo('Orienting')

        rospy.sleep(1)
        e = subprocess.Popen(
            "rosrun mbzirc_c2_auto wrench_detect.py", shell=True)
        rospy.sleep(5)
        d = subprocess.Popen(
            "rosrun mbzirc_c2_auto orient_scan.py", shell=True)
        rospy.sleep(1)
        c = subprocess.Popen(
            "rosrun mbzirc_c2_auto orient2.py", shell=True)


        c.wait()
        rospy.loginfo('Completed Orientation')
        d.kill()
        e.kill()

        return 'oriented'
