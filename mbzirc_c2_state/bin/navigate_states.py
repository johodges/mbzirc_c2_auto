import rospy
import smach
import subprocess


class FindBoard(smach.State):
    """Searches for and then navigates to the board

    Outcomes
    --------
      atBoard : at the board location

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['atBoard'])

    def execute(self, userdata):

        rospy.loginfo('Searching for board')
        a = subprocess.Popen("rosrun mbzirc_c2_auto findbox.py", shell=True)
        b = subprocess.Popen("rosrun mbzirc_c2_auto autonomous.py", shell=True)

        b.wait()
        rospy.loginfo('Searching for board')
        a.kill()

        return 'atBoard'

