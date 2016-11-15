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
import numpy as np

class DriveToValve(smach.State):
    """Moves the arm to stow position and centers the base of the
       UGV in front of the valve.

    Outcomes
    --------
        atValveDrive : at the ready position
        moveStuck : move failed but still retrying
        moveFailed : move failed too many times

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atValveDrive',
                                       'stowArmFailed',
                                       'moveFailed'])

    def execute(self, userdata):
        stow_pos = rospy.get_param('stow_position')
        rospy.set_param('ee_position', [float(stow_pos[0]),
                                        float(stow_pos[1]),
                                        float(stow_pos[2])])

        prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
        prc.wait()

        move_arm_state = rospy.get_param('move_arm_status')

        if move_arm_state == 'success':
            prc = subprocess.Popen("rosrun mbzirc_c2_auto drive2valve.py", shell=True)
            prc.wait()
            smach_state = rospy.get_param('smach_state')

            if smach_state == 'valvepos':
                return 'atValveDrive'
            else:
                return 'moveFailed'


        else:
            return 'stowArmFailed'



class MoveToValveReady(smach.State):
    """Moves the arm in front of valve for detection

    Outcomes
    --------
        atValveReady : at the ready position
        moveStuck : move failed but still retrying
        moveFailed : move failed too many times

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atValveReady',
                                       'moveStuck',
                                       'moveFailed'],
                             input_keys=['move_counter_in',
                                         'max_retries'],
                             output_keys=['move_counter_out'])

    def execute(self, userdata):

        valve_ID_ready_pos = rospy.get_param('valve')

        valve_ID_ready_pos[0] = valve_ID_ready_pos[0]-0.5
        valve_ID_ready_pos[2] = valve_ID_ready_pos[2]+0.1

        rospy.set_param('ee_position', [float(valve_ID_ready_pos[0]),
                                        float(valve_ID_ready_pos[1]),
                                        float(valve_ID_ready_pos[2])])

        prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
        prc.wait()

        move_state = rospy.get_param('move_arm_status')

        # Preset the out move counter to 0, override if necessary
        userdata.move_counter_out = 0

        if move_state == 'success':
            return 'atValveReady'


        else:
            if userdata.move_counter_in < userdata.max_retries:
                userdata.move_counter_out = userdata.move_counter_in + 1
                return 'moveStuck'

            else:
                return 'moveFailed'




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
        moveStuck : move failed but still retrying
        moveFailed : move failed too many times

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atValve',
                                       'centerValve',
                                       'moveStuck',
                                       'moveFailed'],
                             input_keys=['move_counter_in',
                                        'max_retries'],
                             output_keys=['move_counter_out'])

    def execute(self, userdata):

        valve_ID = rospy.get_param('valve_ID')
        valve_ID_ready_pos = rospy.get_param('valve')

        if np.power(valve_ID[1]*valve_ID[1]+valve_ID[2]*valve_ID[2],0.5) < 0.01:
            return 'atValve'
        else:
            valve_ID_ready_pos[0] = valve_ID_ready_pos[0]-0.5
            valve_ID_ready_pos[1] = valve_ID_ready_pos[1]+0.5*valve_ID[1]
            valve_ID_ready_pos[2] = valve_ID_ready_pos[2]+0.5*valve_ID[2]

            rospy.set_param('ee_position', [float(valve_ID_ready_pos[0]),
                                            float(valve_ID_ready_pos[1]),
                                            float(valve_ID_ready_pos[2])])

            prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
            prc.wait()

            move_state = rospy.get_param('move_arm_status')

            # Preset the out move counter to 0, override if necessary
            userdata.move_counter_out = 0

            if move_state == 'success':
                return 'centerValve'

            else:
                if userdata.move_counter_in < userdata.max_retries:
                    userdata.move_counter_out = userdata.move_counter_in + 1
                    return 'moveStuck'

                else:
                    return 'moveFailed'



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


