#! /usr/bin/env python
import rospy, sys, roslib, rosparam
import moveit_commander
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy, JointState
from moveit_msgs.msg import RobotState
from actionlib_msgs.msg import *
from control_msgs.msg import FollowJointTrajectoryActionResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def callback(data):
    twist = Twist()
    c = data.buttons[4]
    twist.linear.x = 4*c*data.axes[1]
    a = 4*data.axes[1]
    twist.angular.z = 4*c*data.axes[0]
    b = 4*data.axes[0]
    pub.publish(twist)
    
    print a, b, c

# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    #pub = rospy.Publisher('arm_controller', JointTrajectory)
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2UR5')
    rospy.spin()

if __name__ == '__main__':
    start()

