""" test_states.py - Version 1.0 2016-11-10

    This file provides for testing states that can be used to test general functionality.

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
import math


class TestArm(smach.State):
    """Test the arm stability

    Outcomes
    --------
        armTestComplete : arm tested

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['armTestComplete',
                                       'armTestFailed'])

    def execute(self, userdata):

        # test_angles = [0, math.pi/6, math.pi/4, math.pi/3, math.pi/2]
        test_angles = [0, math.pi/8, math.pi/7]

        rospy.loginfo('**** Testing the Arm ****')

        rospy.loginfo('Obtaining the initial location of the arm')
        stow_pos = rospy.get_param('stow_position')
        x0 = stow_pos[0]
        y0 = stow_pos[1]
        z0 = stow_pos[2]
        rospy.loginfo("Initial arm location: %s", " ".join(str(x) for x in stow_pos))

        arm_test_cnt = 0

        rospy.loginfo('==== Left Arm Movement Tests ====')
        # Testing arm movements to the left
        for ta in test_angles:
            arm_test_cnt += 1

            xp = x0 + (0.7 * math.cos(ta))
            yp = y0 + (0.7 * math.sin(ta))
            zp = z0 - 0.2

            rospy.set_param('ee_position', [float(xp), float(yp), float(zp)])

            rospy.loginfo('==> Left Arm Test %s', str(arm_test_cnt))
            rospy.loginfo('Target Arm Location: [%s, %s, %s]', str(xp), str(yp), str(zp))

            prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
            prc.wait()

            move_state = rospy.get_param('move_arm_status')

            if move_state in 'failure':
                rospy.loginfo('*****  ARM TEST FAILED  *****')
                return 'armTestFailed'


        rospy.loginfo('Completed Left Arm Movement Tests')
        rospy.loginfo('Stowing the arm.')
        rospy.set_param('ee_position', [float(x0), float(y0), float(z0)])
        prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
        prc.wait()

        arm_test_cnt = 0

        rospy.loginfo('==== Right Arm Movement Tests ====')
        # Testing arm movements to the right
        for ta in test_angles:
            arm_test_cnt += 1

            xp = x0 + (0.7 * math.cos(ta))
            yp = y0 - (0.7 * math.sin(ta))
            zp = z0 - 0.2

            rospy.set_param('ee_position', [float(xp), float(yp), float(zp)])

            rospy.loginfo('==> Right Arm Test %s', str(arm_test_cnt))
            rospy.loginfo('Target Arm Location: [%s, %s, %s]', str(xp), str(yp), str(zp))

            prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
            prc.wait()

            move_state = rospy.get_param('move_arm_status')

            if move_state is 'failure':
                rospy.loginfo('*****  ARM TEST FAILED  *****')
                return 'armTestFailed'

        rospy.loginfo('Completed Right Arm Movement Tests')
        rospy.loginfo('Stowing the arm.')
        rospy.set_param('ee_position', [float(xp), float(yp), float(zp)])
        # prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
        # prc.wait()


        return 'armTestComplete'



class TestWrenchGrab(smach.State):
    """Test the wrench grabbing process

    Outcomes
    --------
        wrenchTestComplete : at the board location ready to test the wrench

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wrenchTestComplete',
                                       'wrenchTestFailed'])

    def execute(self, userdata):
        rospy.loginfo("*** START WRENCH TEST ***")
        # subprocess.Popen( "rosrun mbzirc_c2_auto wrench_detect.py", shell=True)
        subprocess.Popen( "rosrun mbzirc_c2_auto orient_scan.py", shell=True)
        rospy.sleep(5)
        return 'wrenchTestComplete'



class TestValveOp(smach.State):
    """Searches for and then navigates to the board

    Outcomes
    --------
        atBoard : at the board location

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['valveOpTestComplete',
                                       'valveOpTestFailed'])

    def execute(self, userdata):
        rospy.loginfo("*** START VALVE TEST ***")
        return 'valveOpTestComplete'

