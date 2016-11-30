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
from geometry_msgs.msg import Twist
from mbzirc_c2_auto.msg import kf_msg

class StowArm(smach.State):
    """Moves the arm to stow position

    Outcomes
    --------
        armStowed : arm is in the stow position
        stowArmFailed : failed to stow the arm

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['armStowed',
                                       'stowArmFailed'])

    def execute(self, userdata):
        stow_pos = rospy.get_param('stow_position')
        rospy.set_param('ee_position', [float(stow_pos[0]),
                                        float(stow_pos[1]),
                                        float(stow_pos[2])])
        move_state = 'SendGoal'; rospy.set_param('move_arm_status',move_state);
        goal_pub = rospy.Publisher("/move_arm/goal",Twist, queue_size=1)
        tw = Twist()
        tw.linear.x = stow_pos[0]
        tw.linear.y = stow_pos[1]
        tw.linear.z = stow_pos[2]
        flag = 0; ct = 0; ct2 = 0
        
        while flag == 0:
            while ct < 5:
                goal_pub.publish(tw)
                ct = ct+1
                rospy.sleep(0.1)
            move_state = rospy.get_param('move_arm_status')
            if move_state == 'moveFailed':
                ct = 0
                ct2 = ct2+1
            if move_state == 'success':
                flag = 1
            if ct2 > 5:
                flag = 1
        #prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
        #prc.wait()

        move_arm_state = rospy.get_param('move_arm_status')
        #move_arm_state = 'success'
        if move_arm_state == 'success':
            return 'armStowed'
        else:
            return 'stowArmFailed'



class DriveToValve(smach.State):
    """Centers the base of the UGV in front of the valve.

    Outcomes
    --------
        atValveDrive : at the ready position
        moveFailed : move failed too many times

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atValveDrive',
                                       'moveFailed'])

    def execute(self, userdata):
        prc = subprocess.Popen("rosrun mbzirc_c2_auto drive2valve.py", shell=True)
        prc.wait()
        smach_state = rospy.get_param('smach_state')
        #smach_state = 'valvepos'

        if smach_state == 'valvepos':
            return 'atValveDrive'
        else:
            return 'moveFailed'



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

        valve_ID_ready_pos = rospy.get_param('valve2')
        rospy.sleep(0.1)
        ee_position = rospy.get_param('ee_position')
        rospy.sleep(0.1)
        valve_ID_ready_pos[0] = valve_ID_ready_pos[0]-0.4
        valve_ID_ready_pos[2] = valve_ID_ready_pos[2]
        print "Valve ID ready pos: ", valve_ID_ready_pos
        print "ee_position: ", ee_position
        rospy.set_param('ee_position', [float(valve_ID_ready_pos[0]),
                                        float(valve_ID_ready_pos[1]),
                                        0.3])
                                        #float(valve_ID_ready_pos[2])])

        rospy.loginfo("Moving to Valve Ready at: %s",
                      " ".join(str(x) for x in valve_ID_ready_pos))

        # rospy.spin()

        move_state = 'SendGoal'; rospy.set_param('move_arm_status',move_state);
        goal_pub = rospy.Publisher("/move_arm/goal",Twist, queue_size=1)
        tw = Twist()
        tw.linear.x = valve_ID_ready_pos[0]
        tw.linear.y = valve_ID_ready_pos[1]
        tw.linear.z = 0.3
        flag = 0; ct = 0; ct2 = 0
        
        while flag == 0:
            while ct < 5:
                goal_pub.publish(tw)
                ct = ct+1
                rospy.sleep(0.1)
            move_state = rospy.get_param('move_arm_status')
            if move_state == 'moveFailed':
                ct = 0
                ct2 = ct2+1
            if move_state == 'success':
                flag = 1
            if ct2 > 5:
                flag = 1

        #prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
        #prc.wait()

        move_state = rospy.get_param('move_arm_status')

        # Preset the out move counter to 0, override if necessary
        userdata.move_counter_out = 0
        #move_state = 'success'

        if move_state == 'success':
            kf = kf_msg()
            kf_pub = rospy.Publisher("/move_arm/kf_init", kf_msg, queue_size=1, latch=True)
            ee_pub = rospy.Publisher("/move_arm/valve_pos", Twist, queue_size=1)
            kf.initial_pos = [0,0,0,0,0,0]
            kf.initial_covar = [0.001,0.001]
            kf.covar_motion = [0.001,0.001]
            kf.covar_observer = [0.001,0.001]

            kf_pub.publish(kf)
            rospy.sleep(0.1) # Wait for 0.1s to ensure init topic is published
            rospy.sleep(1) # Wait for 1.0s to ensure init routine is completed
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
                             outcomes=['valveLocated',
                                       'valveNotFound'],
                             output_keys=['valve_centered_out'])

    def execute(self, userdata):
        prc = subprocess.Popen("rosrun mbzirc_c2_auto idvalve.py", shell=True)
        prc.wait()

        # smach_state will be set to valveNotFound, valveCenter, valveOffCenter
        sm_state = rospy.get_param('smach_state')
        if sm_state == "valveNotFound":
            return 'valveNotFound'
        else:
            if sm_state == 'valveCenter':
                userdata.valve_centered_out = True
            else:
                userdata.valve_centered_out = False

            return 'valveLocated'



class MoveToValve(smach.State):
    """Move to the valve to prepare for video servo

    Outcomes
    --------
        servoArm : servo in to the valve
        moveForward : move in to operate the valve

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['servoArm',
                                       'moveForward'],
                             input_keys=['valve_centered_in'])

    def callback(self, data):
        self.valve_pos_kf = data

    def execute(self, userdata):
        rospy.Subscriber('/move_arm/valve_pos_kf', Twist, self.callback)
        ee_pub = rospy.Publisher("/move_arm/valve_pos", Twist, queue_size=1)
        rospy.sleep(0.1)
        valve = rospy.get_param('valve')
        ee_position = rospy.get_param('ee_position')

        if valve[0] > 0.31 or valve[1] != 0 or valve[2] != 0:
            ee_position[0] = ee_position[0]+0.5*(valve[0]-0.21)
            print "ee_position before Kalman filter", ee_position
            print "************************************************"
            ee_twist = Twist()
            ee_twist.linear.x = ee_position[0]
            ee_twist.linear.y = valve[1]
            ee_twist.linear.z = valve[2]
            ee_pub.publish(ee_twist)
            rospy.sleep(0.1)
            tw = self.valve_pos_kf
            print "Estimated pose from Kalman filter: ", tw

            ee_position[1] = ee_position[1]+tw.linear.y
            ee_position[2] = ee_position[2]+tw.linear.z
            rospy.set_param('ee_position', [float(ee_position[0]),
                                                float(ee_position[1]),
                                                float(ee_position[2])])
            rospy.sleep(0.1)
            print "***********************************************"
            print "ee_position after Kalman filter", ee_position
            ee_twist.linear.y = ee_position[1]
            ee_twist.linear.z = ee_position[2]
            return 'servoArm'

        else:
            return 'moveForward'


class ServoToValve(smach.State):
    """Moves the arm forward and to center the valve in the camera FOV

    Outcomes
    --------
        movedSuccess : completed servo to center the valve in the FOV
        moveFailed : move failed too many times

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['moveSuccess',
                                       'moveFailed'])

    def execute(self, userdata):
        # Execute the arm servo
        ee_position = rospy.get_param('ee_position')
        move_state = 'SendGoal'; rospy.set_param('move_arm_status',move_state);
        goal_pub = rospy.Publisher("/move_arm/goal",Twist, queue_size=1)
        tw = Twist()
        tw.linear.x = ee_position[0]
        tw.linear.y = ee_position[1]
        tw.linear.z = ee_position[2]
        flag = 0; ct = 0; ct2 = 0
        
        while flag == 0:
            while ct < 5:
                goal_pub.publish(tw)
                ct = ct+1
                rospy.sleep(0.1)
            move_state = rospy.get_param('move_arm_status')
            if move_state == 'moveFailed':
                ct = 0
                ct2 = ct2+1
            if move_state == 'success':
                flag = 1
            if ct2 > 5:
                flag = 1

        #prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
        #prc.wait()

        # Return the state outcome

        move_state = rospy.get_param('move_arm_status')
        if move_state == 'success':
            return 'moveSuccess'
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
        valve = rospy.get_param('valve')
        ee_position = rospy.get_param('ee_position')
        diff = (valve[0]+0.461)-ee_position[0]
        print "****************************************"
        print "Diff: ", diff
        if diff < 0.03:
            rospy.set_param('ee_position', [float(ee_position[0]+0.005),
                                            float(ee_position[1]),
                                            float(ee_position[2])])
            prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
            prc.wait()
            return 'wrenchOnValve'
        else:
            return 'wrenchOnValve'
        #prc = subprocess.Popen("rosrun mbzirc_c2_auto move2op.py", shell=True)
        #prc.wait()
        #return rospy.get_param('smach_state')


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
        prc = subprocess.Popen("rosrun mbzirc_grasping rotate_valve.py", shell=True)
        prc.wait()

        return rospy.get_param('smach_state')


