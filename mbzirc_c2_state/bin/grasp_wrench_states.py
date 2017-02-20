""" grasp_wrench_states.py - Version 1.0 2016-11-10

    State machine classes for grasping a certain wrench from a peg board.

    Classes:
        MoveToReady -
        MoveToWrenchReady
        IDWrench
        MoveToWrench
        MoveToGrasp
        GraspWrench

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
from geometry_msgs.msg import Twist
from mbzirc_c2_auto.msg import kf_msg
from robot_mv_cmds import *
from std_msgs.msg import Int8

class MoveToReady(smach.State):
    """Moves the arm to the ready state from the stowed state

    Outcomes
    --------
        atReady : at the ready position
        moveFailed : move failed too many times
    """

    def __init__(self):
        '''MoveToReady initialization'''
        smach.State.__init__(self,
                             outcomes=['atReady',
                                       'moveFailed'])

    def execute(self, userdata):
        '''MoveToReady execution routine'''
        #
        curr_pos = rospy.get_param('ee_position')
        curr_pos[0] = curr_pos[0] + 0.2
        curr_pos[2] = curr_pos[2] + 0.2
        rospy.set_param('ee_position', [float(curr_pos[0]),
                                        float(curr_pos[1]),
                                        float(curr_pos[2])])
        rospy.set_param('move_arm_status','Pending')
        move_state = moveArmTwist(curr_pos[0], curr_pos[1], curr_pos[2])
        gripper_pub_phys = rospy.Publisher('gripper_req', Int8, queue_size = 1)
        gripper_pub_phys.publish(int(1))
        if move_state == 'success':
            return 'atReady'

        else:
            return 'moveFailed'




class MoveToWrenchReady(smach.State):
    """Moves the arm into position for identifying the wrench

    Outcomes
    --------
        atWrenchReady : at location to determine correct wrench
        moveToOperate :

        moveStuck : move failed but still retrying
        moveFailed : move failed too many times

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atWrenchReady',
                                       'moveToOperate',
                                       'moveStuck',
                                       'moveFailed'],
                             input_keys=['move_counter_in',
                                         'max_retries',
                                         'got_wrench'],
                             output_keys=['move_counter_out'])

    def execute(self, userdata):

        wrench_ready_pos = rospy.get_param('wrench')
        ee_position = rospy.get_param('ee_position')
        
        # Set the ready position 40 cm away from the wrenches
        wrench_ready_pos[0] = (wrench_ready_pos[0] - ee_position[0] - 0.5)+ee_position[0]
        wrench_ready_pos[1] = wrench_ready_pos[1] #+ 0.1
        wrench_ready_pos[2] = 0.3 #wrench_ready_pos[2] - 0.05

        rospy.set_param('ee_position', [float(wrench_ready_pos[0]),
                                        float(wrench_ready_pos[1]),
                                        float(wrench_ready_pos[2])])
        rospy.set_param('move_arm_status','Pending')
        move_state = moveArmTwist(wrench_ready_pos[0], wrench_ready_pos[1], wrench_ready_pos[2])
        # Preset the out move counter to 0, override if necessary
        userdata.move_counter_out = 0


        if move_state == 'success':
            if userdata.got_wrench is True:
                return 'moveToOperate'
            else:
                return 'atWrenchReady'


        else:
            if userdata.move_counter_in < userdata.max_retries:
                userdata.move_counter_out = userdata.move_counter_in + 1
                return 'moveStuck'

            else:
                return 'moveFailed'



class IDWrench(smach.State):
    """ID the correct wrench

    Outcomes
    --------
        wrenchNotFound : unable to locate the correct wrench
        wrenchFound : located the correct wrench

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wrenchFound',
                                       'armTest',
                                       'wrenchNotFound'])

    def execute(self, userdata):
        try:
            physical_robot = rospy.get_parm('physical_robot')
        except:
            physical_robot = 'false'
        if physical_robot == 'false':
            prc = subprocess.Popen("rosrun mbzirc_c2_auto idwrench2.py", shell=True)
            prc.wait()
        if physical_robot == 'True':
            prc = subprocess.Popen("rosrun mbzirc_c2_auto idwrench_phys.py", shell=True)
            prc.wait()
        ret_state = rospy.get_param('smach_state')
        return ret_state




class MoveToWrench(smach.State):
    """Move in front of correct wrench to servo in to wrench

    Outcomes
    --------
        atWrench : in front of wrench
        moveStuck : move failed but still retrying
        moveFailed : move failed too many times

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atWrench',
                                       'moveStuck',
                                       'moveFailed'],
                             input_keys=['move_counter_in',
                                         'max_retries'],
                             output_keys=['move_counter_out'])

    def callback(self, data):
        self.valve_pos_kf = data
    
    def callback_gripper(self, data):
        self.gripper_status = data.data

    def execute(self, userdata):
        rospy.Subscriber('/move_arm/valve_pos_kf', Twist, self.callback)
        rospy.Subscriber('/gripper_status', Int8, self.callback_gripper)
        rospy.sleep(0.1)
        wrench_id = rospy.get_param('wrench_ID_m')
        rospy.sleep(0.1)
        ee_position = rospy.get_param('ee_position')
        rospy.logdebug("wrench_id: [%f, %f, %f]", wrench_id[0], wrench_id[1], wrench_id[2])
        rospy.logdebug("ee_position: [%f, %f, %f]",
                       ee_position[0],
                       ee_position[1],
                       ee_position[2])
        # Set the ready position 40 cm away from the wrenches
        safety_dist = 0.40
        wrench_id[0] = ee_position[0]+wrench_id[0]-safety_dist
        wrench_id[1] = ee_position[1]+wrench_id[1]
        wrench_id[2] = ee_position[2]+wrench_id[2]+0.1

        rospy.set_param('ee_position', [float(wrench_id[0]),
                                        float(wrench_id[1]),
                                        float(wrench_id[2])])
        rospy.set_param('move_arm_status','Pending')
        move_state = moveArmTwist(wrench_id[0], wrench_id[1], wrench_id[2])
        rospy.logdebug("Arm is centered on the wrench and 0.40m off the board.")
        rospy.sleep(1)
        dist_to_move = 0.0
        try:
            physical_robot = rospy.get_param('physical_robot')
        except:
            physical_robot = 'false'
        rospy.sleep(0.1)
        if True: #physical_robot == 'false':
            moveUGVvel(0.05,0.05,'linear')
        move_state = rospy.get_param('move_arm_status')
        #wrench_id[0] = safety_dist #wrench_id[0]-dist_to_move
        rospy.set_param('wrench_ID_m',wrench_id)
        if True: #physical_robot == 'false':
            """
            Open the gripper
            Simulation
            """
            gripper_pub = rospy.Publisher('gripper/cmd_vel', Twist, queue_size = 1)
            twist = Twist()
            speed = .5; turn = 1
            x = 0; y = 0; z = 0;
            th = 1 # To open gripper (1) use th = 1
            twist.linear.x = x*speed;
            twist.linear.y = y*speed;
            twist.linear.z = z*speed;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = th*turn

            ct = 0
            rest_time = 0.1
            tot_time = 3

            while ct*rest_time < tot_time:
                gripper_pub.publish(twist)
                rospy.sleep(0.1)
                ct = ct+1
        if physical_robot == 'True':
            """
            Open the gripper
            Physical
            """
            gripper_pub_phys = rospy.Publisher('gripper_req', Int8, queue_size = 1)
            gripper_pub_phys.publish(int(1))
            rospy.sleep(0.1)
            gripper_status2 = 0

        # Preset the out move counter to 0, override if necessary
        userdata.move_counter_out = 0

        if move_state == 'success':
            rospy.sleep(0.1)
            wrench_id = rospy.get_param('wrench_ID_m')
            wrench_id[0]
            rospy.logdebug("Wrench ID: [%f, %f, %f]", wrench_id[0], wrench_id[1], wrench_id[2])
            thresh = 10
            xA = 20
            ee_position = rospy.get_param('ee_position')
            ct3 = 0
            ee_position[0] = ee_position[0]+0.1

            kf = kf_msg()
            kf_pub = rospy.Publisher("/move_arm/kf_init", kf_msg, queue_size=1, latch=True)
            ee_pub = rospy.Publisher("/move_arm/valve_pos", Twist, queue_size=1)
            kf.initial_pos = [0,0,0,0,0,0]
            kf.initial_covar = [0.001,0.001]
            kf.covar_motion = [0.001,0.001]
            kf.covar_observer = [0.01,0.01]

            kf_pub.publish(kf)
            rospy.sleep(0.1) # Wait for 0.1s to ensure init topic is published
            rospy.sleep(1) # Wait for 1.0s to ensure init routine is completed
            ee_twist = Twist()
            if physical_robot == 'false':
                self.gripper_status = 0
            self.gripper_status = 0

            while self.gripper_status == 0 or self.gripper_status == 4:
                #print "Gripper status: ", self.gripper_status
                rospy.sleep(0.1)
                rospy.loginfo("Gripper Status:")
                rospy.loginfo(self.gripper_status)
                try:
                    physical_robot = rospy.get_parm('physical_robot')
                except:
                    physical_robot = 'false'
                if physical_robot == 'false':
                    prc = subprocess.Popen("rosrun mbzirc_c2_auto centerwrench_tip.py", shell=True)
                    prc.wait()
                if physical_robot == 'True':
                    prc = subprocess.Popen("rosrun mbzirc_c2_auto centerwrench_phys.py", shell=True)
                    prc.wait()


                rospy.sleep(0.1)
                dist = rospy.get_param("wrench_ID_dist")
                rospy.sleep(0.1)
                wrench_id = rospy.get_param('wrench_ID_m')
                rospy.sleep(0.1)
                ee_position = rospy.get_param('ee_position')
                rospy.sleep(0.1)
                wrench_id_px = rospy.get_param('wrench_ID')
                rospy.sleep(0.1)
                rospy.logdebug("wrench_id: [%f, %f, %f]", wrench_id[0], wrench_id[1], wrench_id[2])
                rospy.logdebug("Wrench_id_px: [%f, %f]",
                               wrench_id_px[0],
                               wrench_id_px[1])
                dx = wrench_id[0]*0.05
                xA = rospy.get_param('xA')
                # Set the ready position 40 cm away from the wrenches
                ee_position[0] = (xA + 0.4-0.14)+0.005*ct3 # 0.134 distance from camera to left_tip
                rospy.logdebug("ee_position before Kalman filter: [%f, %f, %f]",
                               ee_position[0],
                               ee_position[1],
                               ee_position[2])
                rospy.logdebug("************************************************")
                ee_twist.linear.x = ee_position[0]
                ee_twist.linear.y = wrench_id[1]
                ee_twist.linear.z = wrench_id[2]
                ee_pub.publish(ee_twist)
                rospy.sleep(0.1)
                tw = self.valve_pos_kf
                rospy.logdebug("Estimated pose from Kalman filter:")
                rospy.logdebug("x = %f", tw.linear.x)
                rospy.logdebug("y = %f", tw.linear.x)
                rospy.logdebug("z = %f", tw.linear.x)

                #ee_position[0] = ee_position[0]+0.005 #
                ee_position[1] = ee_position[1]+tw.linear.y
                ee_position[2] = ee_position[2]+tw.linear.z
                rospy.set_param('ee_position', [float(ee_position[0]),
                                                float(ee_position[1]),
                                                float(ee_position[2])])
                rospy.sleep(0.1)
                rospy.set_param('wrench_ID_m',[wrench_id[0]-dx, wrench_id[1], wrench_id[2]])
                rospy.sleep(0.1)
                rospy.logdebug("***********************************************")
                rospy.logdebug("ee_position after Kalman filter: [%f, %f, %f]",
                               ee_position[0],
                               ee_position[1],
                               ee_position[2])
                ee_twist.linear.y = ee_position[1]
                ee_twist.linear.z = ee_position[2]
                rospy.set_param('move_arm_status','Pending')
                move_state = moveArmTwist(ee_twist.linear.x, ee_twist.linear.y, ee_twist.linear.z)

                #prc = subprocess.Popen("rosrun mbzirc_grasping move_arm_param.py", shell=True)
                #prc.wait()
                ct3 = ct3+1
                if True: #physical_robot == 'False':
                    rospy.loginfo("***********************************************")
                    rospy.loginfo("ee_position:")
                    rospy.loginfo(ee_position[0])
                    rospy.loginfo("threshold:")
                    rospy.loginfo(xA+0.461-0.170)
                    rospy.loginfo("***********************************************")
                    if ee_position[0] < xA+0.461-0.170:
                        self.gripper_status = 0
                    else:
                        self.gripper_status = 1
            rospy.logdebug("We are close enough!")
            rospy.loginfo("Gripper is closing. Waiting for complete signal.")
            ct4 = 0
            if True: #physical_robot == 'false':
                twist = Twist()
                speed = .5; turn = 1
                x = 0; y = 0; z = 0;
                th = -1 # To close gripper (1) use th = 1
                twist.linear.x = x*speed;
                twist.linear.y = y*speed;
                twist.linear.z = z*speed;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = th*turn
                ct = 0
                rest_time = 0.1
                tot_time = 3

                while ct*rest_time < tot_time:
                    gripper_pub.publish(twist)
                    rospy.sleep(0.1)
                    ct = ct+1
                self.gripper_status = 2
            while self.gripper_status != 2:
                rospy.sleep(0.1)
                rospy.logdebug("Gripper Status:")
                rospy.logdebug(self.gripper_status)
                ct4 = ct4+1
                if ct4 > 100:
                    """
                    gripper_pub_phys = rospy.Publisher('gripper_req', Int8, queue_size = 1)
                    gripper_pub_phys.publish(int(0))
                    rospy.sleep(1)
                    gripper_pub_phys.publish(int(1))
                    rospy.sleep(1)
                    ct4 = 0
                    """
                    self.gripper_status = 2
                    
            rospy.loginfo("Gripper is closed.")

            return 'atWrench'

        else:
            if userdata.move_counter_in < userdata.max_retries:
                userdata.move_counter_out = userdata.move_counter_in + 1
                return 'moveStuck'

            else:
                return 'moveFailed'

class MoveToGrasp(smach.State):
    """Video servo to the grasp position

    Outcomes
    --------
        readyToGrasp - in position for the gripper to grab wrench

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['readyToGrasp'])

    def execute(self, userdata):
        prc = subprocess.Popen("rosrun mbzirc_c2_auto move2grasp.py", shell=True)
        prc.wait()
        return rospy.get_param('smach_state')



class GraspWrench(smach.State):
    """Close the gripper

    Outcomes
    --------
        wrenchGrasped - Grabbed the wrench
        gripFailure - failed to grab the wrench

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wrenchGrasped',
                                       'gripFailure',
                                       'wrenchTestDone'],
                             input_keys=['sim_type_in'],
                             output_keys=['got_wrench'])

    def execute(self, userdata):
        try:
            physical_robot = rospy.get_param('physical_robot')
        except:
            physical_robot = 'false'
        if True: #physical_robot == 'false':
            prc = subprocess.Popen("rosrun mbzirc_c2_auto grasp.py", shell=True)
            prc.wait()
        prc = subprocess.Popen("rosrun mbzirc_c2_auto mask_wrench.py", shell=True)
        prc.wait()
        rospy.sleep(0.1)
        ee_position = rospy.get_param('ee_position')
        ee_position[0] = ee_position[0]-0.25
        rospy.set_param('ee_position', [float(ee_position[0]),
                                        float(ee_position[1]),
                                        float(ee_position[2])])

        move_state = moveArmTwist(ee_position[0], ee_position[1], ee_position[2])
        rospy.sleep(0.1)

        rospy.set_param('smach_state','wrenchGrasped')
        rospy.sleep(0.1)
        status = rospy.get_param('smach_state')

        if status == 'wrenchGrasped':
            userdata.got_wrench = True

        if userdata.sim_type_in == 'normal':
            return status
        else:
            return 'wrenchTestDone'


