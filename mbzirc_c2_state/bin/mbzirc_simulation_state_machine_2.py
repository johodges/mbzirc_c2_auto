#!/usr/bin/env python

# import roslib
import rospy
import smach
import smach_ros
# import time
# import os
# import subprocess
from navigate_states import *
from orient_states import *
from grasp_wrench_states import *
from operate_valve_states import *
# from geometry_msgs.msg import Twist
# import math
# import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
# from sensor_msgs.msg import JointState

# JOINT_NAMES = ['ur5_arm_shoulder_pan_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_elbow_joint',
#                'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint']

# *************************************************************************
# State classes are defined in files associated with the sub-state machine
#
# navigation_states.py
#   FindBoard
#
# orient_states.py
#   Orient
#
# grasp_wrench_states.py
#   MoveToReady
#   MoveToReadyWreanch
#   IDWrench
#   MoveToWrench
#   MoveToGrasp
#   GraspWrench
#
# operate_valve_states.py
#   MoveToValveReady
#   IDValve
#   MoveToValve
#   MoveToOperate
#   RotateValve
#
# *************************************************************************

# class FindBoard(smach.State):
#     """Searches for and then navigates to the board

#     Outcomes
#     --------
#       atBoard : at the board location

#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['atBoard'])

#     def execute(self, userdata):

#         rospy.loginfo('Searching for board')
#         a = subprocess.Popen("rosrun mbzirc_c2_auto findbox.py", shell=True)
#         b = subprocess.Popen("rosrun mbzirc_c2_auto autonomous.py", shell=True)

#         b.wait()
#         rospy.loginfo('Searching for board')
#         a.kill()

#         return 'atBoard'




def main():
    """Defines the state machines for Smach
    """

    rospy.init_node('mbzirc_simulation_state_machine', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # Open the container
    with sm:

        # Create the sub SMACH state machine for navigation
        sm_nav = smach.StateMachine(outcomes=['readyToOrient'])

        # Create the sub SMACH state machine for orienting
        sm_orient = smach.StateMachine(outcomes=['readyToGrabWrench'])

        # Create the sub SMACH state machine for grabbing wrench
        sm_wrench = smach.StateMachine(outcomes=['readyToOperate',
                                                 'droppedWrench',
                                                 'wrenchIDFailed'])

        # Create the sub SMACH state machine operating the valve
        sm_valve = smach.StateMachine(outcomes=['valveOperated',
                                                'valveIDFailed',
                                                'lostWrench',
                                                'valveStuck'])

        # Define the NAVIGATE State Machine
        with sm_nav:
            smach.StateMachine.add('FINDBOARD', FindBoard(),
                                   transitions={'atBoard': 'readyToOrient'})

        # Define the ORIENT State Machine
        with sm_orient:
            smach.StateMachine.add('ORIENT_HUSKY', Orient(),
                                   transitions={'oriented': 'readyToGrabWrench'})

        # Define the GRAB_WRENCH State Machine
        with sm_wrench:
            smach.StateMachine.add('MOVE_TO_READY', MoveToReady(),
                                   transitions={'atReady': 'MOVE_WRENCH_READY'})

            smach.StateMachine.add('MOVE_WRENCH_READY', MoveToWrenchReady(),
                                   transitions={'atWrenchReady': 'ID_WRENCH'})

            smach.StateMachine.add('ID_WRENCH', IDWrench(),
                                   transitions={'wrenchFound': 'MOVE_TO_WRENCH',
                                                'wrenchNotFound': 'wrenchIDFailed'})

            smach.StateMachine.add('MOVE_TO_WRENCH', MoveToWrench(),
                                   transitions={'atWrench': 'MOVE_TO_GRASP'})

            smach.StateMachine.add('MOVE_TO_GRASP', MoveToGrasp(),
                                   transitions={'readyToGrasp': 'GRASP_WRENCH'})

            smach.StateMachine.add('GRASP_WRENCH', GraspWrench(),
                                   transitions={'wrenchGrasped': 'readyToOperate',
                                                'gripFailure': 'droppedWrench'})

        # Define the OPERATE_VALVE State Machine
        with sm_valve:
            smach.StateMachine.add('MOVE_VALVE_READY', MoveToValveReady(),
                                   transitions={'atValveReady': 'ID_VALVE'})

            smach.StateMachine.add('ID_VALVE', IDValve(),
                                   transitions={'valveFound': 'MOVE_TO_VALVE',
                                                'valveNotFound': 'valveIDFailed'})

            smach.StateMachine.add('MOVE_TO_VALVE', MoveToValve(),
                                   transitions={'atValve': 'MOVE_TO_OPERATE'})

            smach.StateMachine.add('MOVE_TO_OPERATE', MoveToOperate(),
                                   transitions={'wrenchFell': 'lostWrench',
                                                'wrenchOnValve': 'ROTATE_VALVE'})

            smach.StateMachine.add('ROTATE_VALVE', RotateValve(),
                                   transitions={'wrenchFell': 'lostWrench',
                                                'cantTurnValve': 'valveStuck',
                                                'turnedValve': 'valveOperated'})

        # Add containers to the state
        smach.StateMachine.add('NAVIGATE', sm_nav,
                               transitions={'readyToOrient': 'ORIENT'})

        smach.StateMachine.add('ORIENT', sm_orient,
                               transitions={'readyToGrabWrench': 'GRAB_WRENCH'})

        smach.StateMachine.add('GRAB_WRENCH', sm_wrench,
                               transitions={'readyToOperate': 'OPERATE_VALVE',
                                            'droppedWrench': 'failure',
                                            'wrenchIDFailed': 'failure'})

        smach.StateMachine.add('OPERATE_VALVE', sm_valve,
                               transitions={'valveOperated': 'success',
                                            'valveIDFailed': 'failure',
                                            'lostWrench': 'failure',
                                            'valveStuck': 'failure'})


    # Create the introspection server
    sis = smach_ros.IntrospectionServer('mbzirc_server', sm, '/CHALLENGE_TWO')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
