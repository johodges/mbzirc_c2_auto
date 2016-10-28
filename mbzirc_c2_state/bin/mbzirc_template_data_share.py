#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define states
class FindBoard(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['foundBoard','notFound'],
				input_keys=['findCounterIn'],
				output_keys=['findCounterOut'])

    def execute(self, userdata):
        rospy.loginfo('Searching for board')
        if userdata.findCounterIn == 1:
            return 'foundBoard'
        else:
	    userdata.findCounterOut = userdata.findCounterIn + 1
            return 'notFound'

class Orient(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['oriented','notOriented'],
				input_keys=['orientCounterIn'],
				output_keys=['orientCounterOut'])

    def execute(self, userdata):
        rospy.loginfo('Orienting')
        if userdata.orientCounterIn == 3:
            return 'oriented'
        else:
	    userdata.orientCounterOut = userdata.orientCounterIn + 1
            return 'notOriented'

class WrenchDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['detected','notDetected'],
				input_keys=['detectCounterIn'],
				output_keys=['detectCounterOut'])

    def execute(self, userdata):
        rospy.loginfo('Finding wrench')
        if userdata.detectCounterIn == 5:
            return 'detected'
        else:
	    userdata.detectCounterOut = userdata.detectCounterIn + 1
            return 'notDetected'

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['graspedWrench', 'notGrasped', 'dropped'],
				input_keys=['graspCounterIn'],
				output_keys=['graspCounterOut'])

    def execute(self, userdata):
        rospy.loginfo('Grasping wrench')
        if userdata.graspCounterIn == 8:
            return 'graspedWrench'
        elif userdata.graspCounterIn == 7:
	    userdata.graspCounterOut = userdata.graspCounterIn + 1
            return 'dropped'
	else:
	    userdata.graspCounterOut = userdata.graspCounterIn + 1
	    return 'notGrasped'

class GetDroppedWrench(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['recoveredWrench','dropped'],
				input_keys=['dropCounterIn'],
				output_keys=['dropCounterOut'])

    def execute(self, userdata):
        rospy.loginfo('Recovering')
        if userdata.dropCounterIn > 10:
            return 'recoveredWrench'
        else:
	    userdata.dropCounterOut = userdata.dropCounterIn + 1
            return 'dropped'

class UseWrench(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
				outcomes=['succeeded','dropped','missed'],
				input_keys=['wrenchCounterIn'],
				output_keys=['wrenchCounterOut'])

    def execute(self, userdata):
        rospy.loginfo('Opening valve')
        if userdata.wrenchCounterIn > 11:
            return 'succeeded'
        elif userdata.wrenchCounterIn == 11:
	    userdata.wrenchCounterOut = userdata.wrenchCounterIn + 1
            return 'dropped'
	else:
	    userdata.wrenchCounterOut = userdata.wrenchCounterIn + 1
	    return 'missed'

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FINDBOARD', FindBoard(), 
                               transitions={'foundBoard':'ORIENT', 
                                            'notFound':'FINDBOARD'},
			       remapping={'findCounterIn':'sm_counter',
					  'findCounterOut':'sm_counter'})

        smach.StateMachine.add('ORIENT', Orient(), 
                               transitions={'oriented':'WRENCHDETECTION', 
                                            'notOriented':'ORIENT'},
			       remapping={'orientCounterIn':'sm_counter',
					  'orientCounterOut':'sm_counter'})

        smach.StateMachine.add('WRENCHDETECTION', WrenchDetection(), 
                               transitions={'detected':'GRASP', 
                                            'notDetected':'WRENCHDETECTION'},
			       remapping={'detectCounterIn':'sm_counter',
					  'detectCounterOut':'sm_counter'})

        smach.StateMachine.add('GRASP', Grasp(), 
                               transitions={'graspedWrench':'USEWRENCH', 
                                            'dropped':'GETDROPPEDWRENCH',
					    'notGrasped':'GRASP'},
			       remapping={'graspCounterIn':'sm_counter',
					  'graspCounterOut':'sm_counter'})

        smach.StateMachine.add('GETDROPPEDWRENCH', GetDroppedWrench(), 
                               transitions={'recoveredWrench':'USEWRENCH', 
                                            'dropped':'GETDROPPEDWRENCH'},
			       remapping={'dropCounterIn':'sm_counter',
					  'dropCounterOut':'sm_counter'})

        smach.StateMachine.add('USEWRENCH', UseWrench(), 
                               transitions={'succeeded':'success', 
					    'dropped':'GETDROPPEDWRENCH', 
                                            'missed':'USEWRENCH'},
			       remapping={'wrenchCounterIn':'sm_counter',
					  'wrenchCounterOut':'sm_counter'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
