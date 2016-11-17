#!/usr/bin/env python
# license removed for brevity
# temperate node to publish location of wrench to test correction step
   
import rospy
from geometry_msgs.msg import PoseArray, PointStamped, PoseStamped
    
def circle():
    circle = PointStamped()
    pub = rospy.Publisher('/circle', PointStamped, queue_size = 10)
    rospy.init_node('circle', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    circle = PointStamped()
    circle.header.frame_id = "camera_frame_id"
    circle.header.stamp = rospy.Time.now()
    circle.point.x = 760 #ion = [1.05, -0.08, 0.465]
    circle.point.y = 280
    circle.point.z = 300

    while not rospy.is_shutdown():
        rospy.loginfo("circle detected")
        pub.publish(circle)
        rate.sleep()
   
if __name__ == '__main__':
   try:
         circle()
   except rospy.ROSInterruptException:
       pass
