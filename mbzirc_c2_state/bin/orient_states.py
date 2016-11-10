import rospy
import smach
import subprocess


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
        rospy.loginfo('Orienting')

        c = subprocess.Popen(
            "rosrun mbzirc_c2_auto orient2.py", shell=True)
        d = subprocess.Popen(
            "rosrun mbzirc_c2_auto orient_scan.py", shell=True)
        e = subprocess.Popen(
            "rosrun mbzirc_c2_auto wrench_detect.py", shell=True)

        c.wait()
        rospy.loginfo('Completed Orientation')
        d.kill()
        e.kill()

        return 'oriented'
