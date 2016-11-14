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
from geometry_msgs.msg import PoseStamped
    
def locations():
    pub = rospy.Publisher('/locations', JointState, queue_size=10)
    rospy.init_node('locations', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    locations = JointState()
    locations.position = [1.05, -0.08, 0.465, 0, 0, 0, 1.0]
    #locations.position = [1.05, 0.42, 0.235, 0, 0, 0, 1.0]
    
    while not rospy.is_shutdown():
        rospy.loginfo(locations)
        pub.publish(locations)
        rate.sleep()
   
if __name__ == '__main__': locations()
  
