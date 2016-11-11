""" operate_valve_states.py - Version 1.0 - Created 2016-11-10

    State machine classes for positioning and operating the valve.

    State Classes
        MoveToValveReady -
        IDValve
        MoveToValve
        MoveToOperate
        RotateValve

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

class MoveToValveReady(smach.State):
    """Moves the arm in front of valve for detection

    Outcomes
    --------
        atValveReady : at the ready position

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atValveReady'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'atValveReady'



class IDValve(smach.State):
    """Identifies the center of the valve

    Outcomes
    --------
        valveFound : found the valve
        valveNotFound : could not locate the valve

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['valveFound',
                                       'valveNotFound'])

    def execute(self, userdata):
        prc = subprocess.Popen("rosrun mbzirc_c2_auto idvalve.py", shell=True)
        prc.wait()

        return rospy.get_param('smach_state')



class MoveToValve(smach.State):
    """Move to the valve to prepare for video servo

    Outcomes
    --------
        atValve : at the valve ready to servo in

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atValve'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'atValve'



class MoveToOperate(smach.State):
    """Servo in to valve and place wrench on valve

    Outcomes
    --------
        wrenchFell : wrench fell off the gripper
        wrenchOnValve : at the ready position

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wrenchFell',
                                       'wrenchOnValve'])

    def execute(self, userdata):
        prc = subprocess.Popen("rosrun mbzirc_c2_auto move2op.py", shell=True)
        prc.wait()

        return rospy.get_param('smach_state')


class RotateValve(smach.State):
    """Rotate the valve one full turn

    Outcomes
    --------
        wrenchFell : wrench fell out of the gripper
        cantTurnValve : valve stuck
        turnedValve : able to turn the valve

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wrenchFell',
                                       'cantTurnValve',
                                       'turnedValve'])

    def execute(self, userdata):
        prc = subprocess.Popen("rosrun mbzirc_c2_auto rotate.py", shell=True)
        prc.wait()

        return rospy.get_param('smach_state')


