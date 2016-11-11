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
    
def wloc_pub():
    pub = rospy.Publisher('Location', PoseStamped, queue_size=10)
    rospy.init_node('wrench_loc', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    wlocs = PoseStamped()
    wlocs.point.x = 0.5
    wlocs.point.y = 0.5
    wlocs.point.z = 0.2

    while not rospy.is_shutdown():
        rospy.loginfo(wlocs)
        pub.publish(wlocs)
        rate.sleep()
   
if __name__ == '__main__': wloc_pub()
  
