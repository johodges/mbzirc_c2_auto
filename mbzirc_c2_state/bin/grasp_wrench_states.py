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
    """My description

    Outcomes
    --------
      atWrenchReady : Outcome Reason

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atWrenchReady'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'atWrenchReady'



class IDWrench(smach.State):
    """My description

    Outcomes
    --------
      wrenchFound : Outcome Reason

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wrenchFound'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'wrenchFound'



class MoveToWrench(smach.State):
    """My description

    Outcomes
    --------
      atWrench : Outcome Reason

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atWrench'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'atWrench'



class MoveToGrasp(smach.State):
    """My description

    Outcomes
    --------
      readyToGrasp - Outcome Reason

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['readyToGrasp'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'readyToGrasp'



class GraspWrench(smach.State):
    """My description

    Outcomes
    --------
      wrenchGrasped - Outcome Reason

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wrenchGrasped'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'wrenchGrasped'

