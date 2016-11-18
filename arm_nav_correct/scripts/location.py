#!/usr/bin/env python
# license removed for brevity
# temperate node to publish location of wrench to test correction step
   
import rospy
from geometry_msgs.msg import PoseArray, PointStamped
    
def wrench_loc_pub():
    location = PointStamped()
    pub = rospy.Publisher('/location', PointStamped, queue_size = 10)
    rospy.init_node('wrench_loc_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    location = PointStamped()
    location.header.frame_id = "camera_frame_id"
    location.header.stamp = rospy.Time.now()
    location.point.x = 1.05 #ion = [1.05, -0.08, 0.465]
    location.point.y = -0.08
    location.point.z = 0.465

    while not rospy.is_shutdown():
        rospy.loginfo("wrench loccation published")
        pub.publish(location)
        rate.sleep()
   
if __name__ == '__main__':
   try:
         wrench_loc_pub()
   except rospy.ROSInterruptException:
       pass
