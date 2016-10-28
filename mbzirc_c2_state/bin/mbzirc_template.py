#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define states
class FindBoard(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foundBoard','notFound'])
        self.findBoard = 0

    def execute(self, userdata):
        rospy.loginfo('Searching for board')
        if self.findBoard == 1:
            return 'foundBoard'
        else:
	    self.findBoard = 1
            return 'notFound'

class Orient(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['oriented','notOriented'])
        self.oriented = 0

    def execute(self, userdata):
        rospy.loginfo('Orienting')
        if self.oriented == 1:
            return 'oriented'
        else:
	    self.oriented = 1
            return 'notOriented'

class WrenchDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected','notDetected'])
        self.detected = 0

    def execute(self, userdata):
        rospy.loginfo('Finding wrench')
        if self.detected == 1:
            return 'detected'
        else:
	    self.detected = 1
            return 'notDetected'

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['graspedWrench', 'notGrasped', 'dropped'])
        self.grasped = 0
	self.dropped = 0

    def execute(self, userdata):
        rospy.loginfo('Grasping wrench')
        if self.grasped == 1:
            return 'graspedWrench'
        elif self.dropped:
            return 'dropped'
	else:
	    self.dropped = 1
	    return 'notGrasped'

class GetDroppedWrench(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recoveredWrench','dropped'])
        self.recovered = 0

    def execute(self, userdata):
        rospy.loginfo('Recovering')
        if self.recovered == 1:
            return 'recoveredWrench'
        else:
	    self.recovered = 1
            return 'dropped'

class UseWrench(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','dropped','missed'])
        self.success = 0
	self.dropped = 0

    def execute(self, userdata):
        rospy.loginfo('Opening valve')
        if self.success == 1:
            return 'succeeded'
        elif self.dropped:
            return 'dropped'
	else:
	    self.success = 1
	    return 'missed'

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FINDBOARD', FindBoard(), 
                               transitions={'foundBoard':'ORIENT', 
                                            'notFound':'FINDBOARD'})

        smach.StateMachine.add('ORIENT', Orient(), 
                               transitions={'oriented':'WRENCHDETECTION', 
                                            'notOriented':'ORIENT'})

        smach.StateMachine.add('WRENCHDETECTION', WrenchDetection(), 
                               transitions={'detected':'GRASP', 
                                            'notDetected':'ORIENT'})

        smach.StateMachine.add('GRASP', Grasp(), 
                               transitions={'graspedWrench':'USEWRENCH', 
                                            'dropped':'GETDROPPEDWRENCH',
					    'notGrasped':'GRASP'})

        smach.StateMachine.add('GETDROPPEDWRENCH', GetDroppedWrench(), 
                               transitions={'recoveredWrench':'USEWRENCH', 
                                            'dropped':'GETDROPPEDWRENCH'})

        smach.StateMachine.add('USEWRENCH', UseWrench(), 
                               transitions={'succeeded':'success', 
					    'dropped':'GETDROPPEDWRENCH', 
                                            'missed':'USEWRENCH'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
