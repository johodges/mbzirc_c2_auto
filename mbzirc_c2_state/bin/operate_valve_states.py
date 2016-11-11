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
        rospy.sleep(5)
        return 'valveFound'


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
        rospy.sleep(5)
        return 'wrenchOnValve'


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
        rospy.sleep(5)
        return 'turnedValve'


