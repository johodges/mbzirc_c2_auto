#!/usr/bin/env python
# license removed for brevity
# temperate node for testing publication of published node
   
import rospy
from geometry_msgs.msg import num
    
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "The location of wrench are %s", data.data)

def wrench_loc_list():
    rospy.init_node('wrench_loc_list', anonymous=True)

    rospy.Subscriber("Location", num, callback)

    rospy.spin()

if __name__ == '__main__':
   wrench_loc_list()

