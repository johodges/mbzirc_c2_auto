"""
Function to simplify state machine code
"""

import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

def twistCommand(x, y, z):
	move_state = 'SendGoal'
	rospy.set_param('move_arm_status',move_state)
        goal_pub = rospy.Publisher("/move_arm/goal",Twist, queue_size=1)
        tw = Twist()
        tw.linear.x = x
        tw.linear.y = y
        tw.linear.z = z
        flag = 0
	ct = 0
	ct2 = 0

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

	return move_state
