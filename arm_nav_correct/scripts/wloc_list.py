#!/usr/bin/env python
# license removed for brevity
# temperate node for testing publication of published node
   
import rospy
from sensor_msgs.msg import JointState
    
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "The location of wrench are %s", wlocs.position)

def wloc_list():
    rospy.init_node('wrench_location', anonymous=True)

    rospy.Subscriber("Location", JointState, callback)

    rospy.spin()

if __name__ == '__main__':
   wloc_list()

