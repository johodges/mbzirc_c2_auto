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
import os
import signal
from sensor_msgs.msg import Joy
import sys


# class NavSMMethod(smach.State):
#     """Determine if old or new state machine should be used

#     This should be removed as soon as the new state machine logic for navigation
#     has been implemented

#     Outcomes
#     --------
#         useNew : use the new state machine
#         useOld : use the old state machine
#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['useNew',
#                                        'useOld'])

#     def execute(self, userdata):
#         try:
#             sm_to_use = rospy.get_param('sm_version')
#         except:
#             print sys.exc_info()[0]
#             return 'useOld'

#         if sm_to_use == 'new' or sm_to_use == 'newNavigate':
#             return 'useNew'
#         else:
#             return 'useOld'


class FindBoard(smach.State):
    """Searches for and then navigates to the board

    Outcomes
    --------
        atBoard : at the board location
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atBoard',
                                       'beenToAllWayPoints'])

    def execute(self, userdata):

        ret_state = 'normal'

        rospy.loginfo('Searching for board')
        """
        try:
            lidar_to_use = rospy.get_param('lidar')
        except:
            lidar_to_use = 'sick'
        if lidar_to_use == 'sick':
            a = subprocess.Popen("rosrun mbzirc_c2_auto findbox_2d_vel.py",
                stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            #a = subprocess.Popen("rosrun mbzirc_c2_auto findbox.py", shell=True)
        if lidar_to_use == 'velodyne':
            a = subprocess.Popen("rosrun mbzirc_c2_auto findbox_2d_vel.py",
                stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        """
        physical_robot = rospy.get_param('physical_robot')
        if physical_robot:
            a = subprocess.Popen("rosrun mbzirc_c2_auto findbox_2d_vel.py",
                stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        else:
            a = subprocess.Popen("rosrun mbzirc_c2_auto findbox.py",
                stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        rospy.sleep(1)
        b = subprocess.Popen("rosrun mbzirc_c2_auto autonomous.py", shell=True)


        while b.poll() is None:
            if self.preempt_requested():
                rospy.loginfo("Navigation preempted for manual mode.")
                b.kill()
                rospy.sleep(0.1)
                ret_state = 'preempted'

        if ret_state == 'normal':
            rospy.loginfo('Completed searching for board')
            os.killpg(os.getpgid(a.pid), signal.SIGTERM)
            rospy.sleep(0.1)
            ret_state = rospy.get_param('smach_state')

        return ret_state


class ManNavCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shiftMode','preempted'])
        self.status = 'waiting'
        self.sub = rospy.Subscriber('joy',Joy,self.ck_joystick)

    def ck_joystick(self,msg):
        if msg.buttons[3] == 1:
            self.status = 'shiftMode'
        else:
            self.status = 'waiting'

    def execute(self, userdata):
        rospy.loginfo(self.status)
        while self.status == 'waiting':
            if self.preempt_requested():
                rospy.loginfo('Autonomous navigation successful.  Preempting manual check.')
                self.service_preempt()
                return 'preempted'

        return self.status

class Localize(smach.State):
    """Automatically localize the UGV in the arena

    Use IMU and GPS to localize the UGV in the arena

    Outcomes
    --------
        localized : UGV localized in the arena
        notLocalized : unable to automatically localize the UGV in the arena
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['localized'])

    def execute(self, userdata):
        physical_robot = rospy.get_param('physical_robot')
        vtrex_enabled = rospy.get_param('vtrex_enabled')
        if physical_robot:
            # Launch the UGV
            if vtrex_enabled:
                left_motor = subprocess.Popen("rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                right_motor = subprocess.Popen("rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                odom_publisher = subprocess.Popen("rosrun tele_controller odom_pub_node", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            else:
                husky_pc = subprocess.Popen("rosrun husky_base husky_node _port:=/dev/ttyUSB0", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                husky_con = subprocess.Popen("rosrun controller_manager spawner husky_joint_publisher husky_velocity_controller --shutdown-timeout 3", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            # Launch the imu_localization node to fix yaw
            imu_localization = subprocess.Popen("rosrun mbzirc_c2_auto imu_localization.py", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            imu_localization.wait()

            # Kill odometry nodes
            if vtrex_enabled:
                os.killpg(os.getpgid(left_motor.pid), signal.SIGTERM)
                os.killpg(os.getpgid(right_motor.pid), signal.SIGTERM)
                os.killpg(os.getpgid(odom_publisher.pid), signal.SIGTERM)
                rospy.sleep(0.1)
            else:
                os.killpg(os.getpgid(husky_pc.pid), signal.SIGTERM)
                os.killpg(os.getpgid(husky_con.pid), signal.SIGTERM)
                rospy.sleep(0.1)

            # Launch UGV again to reset odom to zero
            if vtrex_enabled:
                left_motor = subprocess.Popen("rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                right_motor = subprocess.Popen("rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                odom_publisher = subprocess.Popen("rosrun tele_controller odom_pub_node", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            else:
                husky_pc = subprocess.Popen("rosrun husky_base husky_node _port:=/dev/ttyUSB0", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                husky_con = subprocess.Popen("rosrun controller_manager spawner husky_joint_publisher husky_velocity_controller --shutdown-timeout 3", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            rospy.sleep(0.1)

        else:
            pass
        return 'localized'


# class VisualLocalization(smach.State):
#     """Visually localize the UGV in the arena

#     Attempt to use visual localization to orient the UGV in the arena

#     Outcomes
#     --------
#         localized : UGV localized in the environment
#         notLocalized : unable to perform visual localization of the UGV
#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['localized',
#                                        'notLocalized'])

#     def execute(self, userdata):
#         return 'localized'


# class InitializeDetect(smach.State):
#     """Initialize LiDAR and Visual detectors and perform initial arena scan

#     First initialize the LiDAR detector and the PTZ camera for visual detection. Perform
#     an initial scan of the arena to see if the board can be located

#     Outcomes
#     --------
#         boardFound : board found on the initial scan of the arena
#         boardNotFound : unable to locate the board on initial scan
#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['foundBoard',
#                                        'boardNotFound'])

#     def execute(self, userdata):
#         return 'boardFound'


# class MoveToWayPoint(smach.State):
#     """Moves the UGV to the next navigation way point while searching for the board

#     This moves the UGV along a predetermined list of way points that define a search
#     pattern for locating the board.  Additionally it continually checks to see if the
#     board is located by the detector and if it has then it stops and hands the logic
#     off to drive to the board.  If all the way points have been visited then the
#     navigation is turned over to manual operations.

#     Outcomes
#     --------
#         foundBoard : board located while moving to the next way point
#         boardNotFound : at the way point and was unable to find the board
#         beenToAllWayPoints : visited every predefined way point and was unable to
#                              find the board
#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['foundBoard',
#                                        'boardNotFound',
#                                        'beenToAllWayPoints'])

#     def execute(self, userdata):
#         return 'foundBoard'


# class MoveToBoard(smach.State):
#     """Moves the UGV to the predicted board location

#     The location of the board is predicted based on a combination of LiDAR and
#     visual detection.  This state drives the UGV towards the object and continually
#     checks the detectors to ensure that it is on a good course

#     Outcomes
#     --------
#         atBoard : completed the navigation to the board
#         lostObject : lost the object or determined it was the wrong object
#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['atBoard',
#                                        'lostObject'])

#     def execute(self, userdata):
#         return 'atBoard'


class ManualNavigate(smach.State):
    """Manually navigate the UGV to find the tool and valve board

    Shifts the operation to manual if the autonomous routines are unable to locate
    the board after going to all the way points.

    Outcomes
    --------
        atBoard : at the board ready to orient
        noBoard : could not find the board
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atBoard',
                                       'noBoard'])

    def execute(self, userdata):
        prc = subprocess.Popen("roslaunch UGV_teleop manual_control.launch", shell=True)
        prc.wait()

        if rospy.get_param('smach_state') == 'backToAuto':
            return 'atBoard'
        else:
            return 'noBoard'


