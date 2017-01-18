#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    c = data.buttons[4]
    twist.linear.x = 4*c*data.axes[1]
    a = 4*data.axes[1]
    twist.angular.z = 4*c*data.axes[0]
    b = 4*data.axes[0]
    pub.publish(twist)
    
    print a, b, c
    
    global time1, time2, tp1, tp2
    if a!=0:
	time1 = time.time()
	tp1 = rospy.Time.now()
	print "Start time: ", time1
	#print "Start time: ", time1.to_sec()
	print "test1: ", tp1
    if a==0:
	time2 = time.time()
	tp2 = rospy.Time.now()
	print "End time: ", time2
	elapsed = time2 - time1
	print "Elapsed time: ", elapsed
	print "test2: ", tp2
	Diff = tp2 - tp1
	print "Diff: ", Diff
	print "Diff to sec: ", Diff.to_sec()


# Intializes everything
def start():
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist)
    #pub = rospy.Publisher('joy_teleop/cmd_vel', Twist)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()
