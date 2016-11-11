#!/usr/bin/env python
# license removed for brevity
# temperate node to publish location of wrench to test correction step
   
import rospy
import time
import actionlib
import math
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
    
def valoc_pub():
    pub = rospy.Publisher('Location', JointState, queue_size=10)
    rospy.init_node('valve_locs', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    valocs = JointState()
    valocs.position = [0.5, 0.5, 0.5]

    while not rospy.is_shutdown():
        rospy.loginfo(valocs)
        pub.publish(valocs)
        rate.sleep()
   
if __name__ == '__main__': valoc_pub()
  
