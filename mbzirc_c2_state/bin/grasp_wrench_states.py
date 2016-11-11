import rospy
import smach
import subprocess

class MoveToReady(smach.State):
    """Moves the arm
    to the ready state from the stowed state
    Outcomes
    --------
        atReady : at the ready position

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atReady'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'atReady'



class MoveToWrenchReady(smach.State):
    """Moves the arm into position for identifying the wrench

    Outcomes
    --------
        atWrenchReady : at location to determine correct wrench

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atWrenchReady'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'atWrenchReady'



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
                                       'wrenchNotFound'])

    def execute(self, userdata):
        prc = subprocess.Popen("rosrun mbzirc_c2_auto idwrench.py", shell=True)
        prc.wait()

        return rospy.get_param('smach_state')




class MoveToWrench(smach.State):
    """Move in front of correct wrench to servo in to wrench

    Outcomes
    --------
        atWrench : in front of wrench

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atWrench'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'atWrench'



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
                                       'gripFailure'])

    def execute(self, userdata):
        prc = subprocess.Popen("rosrun mbzirc_c2_auto grasp.py", shell=True)
        prc.wait()

        return rospy.get_param('smach_state')

