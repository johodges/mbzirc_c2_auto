#!/usr/bin/env python
# license removed for brevity
# temperate node to publish location of wrench to test correction step
   
import rospy
from geometry_msgs.msg import PoseArray
    
def wrench_loc_pub():
    pub = rospy.Publisher('Location', num)
    rospy.init_node('wrench_loc_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        x_dist = geometry_msgs.x % rospy.get_time()
   	y_dist = geometry_msgs.y
	z_dist = geometry_msgs.z

        rospy.loginfo(x_dist, y_dist, z_dist)
        pub.publish()
        rate.sleep()
   
if __name__ == '__main__':
   try:
         wrench_loc_pub()
   except rospy.ROSInterruptException:
       pass
