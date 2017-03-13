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
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import sys

# class OrientSMMethod(smach.State):
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

#         if sm_to_use == 'new' or sm_to_use == 'newOrient':
#             return 'useNew'
#         else:
#             return 'useOld'



class ManOrientCheck(smach.State):
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
                rospy.loginfo('Autonomous orientation successful.  Preempting manual check.')
                self.service_preempt()
                return 'preempted'

        return self.status

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
        ret_state = 'normal'
        rospy.loginfo('Orienting')
        e = subprocess.Popen(
            "rosrun mbzirc_c2_auto wrench_detect.py", shell=True)
        """
        try:
            lidar_to_use = rospy.get_param('lidar')
        except:
            lidar_to_use = 'sick'
        rospy.sleep(0.1)
        if lidar_to_use == 'sick':
            d = subprocess.Popen(
                "rosrun mbzirc_c2_auto orient_scan_deadzone.py", shell=True)
        if lidar_to_use == 'velodyne':
            d = subprocess.Popen(
                "rosrun mbzirc_c2_auto orient_scan_deadzone.py", shell=True)
        """
        physical_robot = rospy.get_param('physical_robot')
        rospy.sleep(0.1)
        if physical_robot:
            d = subprocess.Popen(
                "rosrun mbzirc_c2_auto orient_scan_deadzone.py", shell=True)
            c = subprocess.Popen("rosrun mbzirc_c2_auto orient2_skid.py", shell=True)
        else:
            d = subprocess.Popen("rosrun mbzirc_c2_auto orient_scan.py", shell=True)
            c = subprocess.Popen("rosrun mbzirc_c2_auto orient2.py", shell=True)
        rospy.sleep(0.1)

        while c.poll() is None:
            if self.preempt_requested():
                rospy.loginfo("Orientation preempted for manual mode.")
                c.kill()
                rospy.sleep(0.1)
                ret_state = 'preempted'

        if ret_state == 'normal':
            rospy.loginfo('Completed Orientation')
            d.kill()
            e.kill()
            ret_state = 'oriented'

        return ret_state



# class GoToNewSide(smach.State):
#     """Moves to a new side of the panel

#     Moves the UGV to the next side of the panel in order to search for the wrenches.

#     Outcomes
#     --------
#         atNewSide : UGV located at the new panel side
#         allSidesScanned : all four sides of the panel have been visited
#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['atNewSide',
#                                        'allSidesScanned'],
#                              input_keys=['num_sides_in'],
#                              output_keys=['num_sides_out'])

#     def execute(self, userdata):
#         userdata.sideCounter_out = userdata.sideCounter_in + 1;

#         if userdata.sideCounter_out > 4:
#             userdata.sideCounter_out = 0
#             rospy.loginfo('All panel sides searched - switching to manual mode.')
#             return 'allSidesScanned'

#         return 'atNewSide'


# class ComputeSideWP(smach.State):
#     """Computes the first way point at a new side of the panel

#     Computes the first way point at a new side.  Essentially this puts the right
#     edge of the panel near the right edge of the camera image.  This often puts
#     the entire panel in the field of view.  If not, then subsequent way points
#     along the side are computed in MOVE_DOWN_SIDE.

#     Outcomes
#     --------
#       computedWayPoint : first way point has been computed
#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['computedWayPoint'])

#     def execute(self, userdata):
#         return 'computedWaypoint'


# class MoveToSideWP(smach.State):
#     """Moves the UGV to the next way point along the side

#     Moves the UGV to the next way point along a given side and then orients the UGV
#     to face the board in order to attempt to detect the wrenches.  The first way
#     point on any side places the right side of the panel on the right edge of the camera
#     field of view.  All subsequent way points move down the panel until the left edge
#     is reached.

#     Outcomes
#     --------
#         atWayPoint : UGV is located at the next way point and oriented facing the panel

#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['atWayPoint'])

#     def execute(self, userdata):
#         return 'atWayPoint'



# class DetectWrenches(smach.State):
#     """Attempts to detect the bank of wrenches in the current camera FOV

#     Searches for the wrenches on the panel.  If found, the initial position is
#     stored in a parameter for later.

#     Outcomes
#     --------
#       oriented : Husky oriented to grasp wrench and turn valve

#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['foundWrenches',
#                                        'noWrenches'])

#     def execute(self, userdata):
#         # Need to call a routing to detect the wrenches here.  wrench_detect.py just
#         # spins so we probably need to Subscribe to its output and determine the
#         # wrench/valve positions here

#         # return rospy.get_param('smach_state')
#         return 'foundWrenches'


# class MoveDownSide(smach.State):
#     """Moves down the wall to check for wrenches

#     This routine first checks to see if the left edge of the panel is in the
#     current view If it is, then this routine returns that it has finished processing
#     the current side. Otherwise, it calculates a new way point (WP) to move the
#     UGV to the left.

#     Outcomes
#     --------
#         finishedSide : left end of the panel is detected
#         nextWayPoint : move to new side way point

#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['finishedSide',
#                                        'nextWayPoint'])

#     def execute(self, userdata):
#         return 'finishedSide'


# class MoveToWrenches(smach.State):
#     """Moves the Husky into position to ID and grab the wrench

#     Move the Husky closer to the board in preparation for grabbing the correct
#     wrench for operating the valve

#     Outcomes
#     --------
#        oriented : Husky oriented to ID and grab wrench

#     """

#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['oriented'])

#     def execute(self, userdata):

#         return 'oriented'


