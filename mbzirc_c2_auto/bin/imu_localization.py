import rospy
import rospkg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
import tf
import math

class imu_localization():
    def __init__(self):
        # Establish ROS node
        rospy.init_node('imu_localization', anonymous=True)
        rospy.on_shutdown(self.shutdown) # Enable shutdown in rospy
        self.rest_time = 0.1            # Minimum pause at each location
        self.twist = Twist()
        self.target_yaw = rospy.get_param('imu_absolute_yaw')

        # Establish publishers and subscribers
        self.twi_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.Subscriber('/imu/data', Imu, self.callback_imu, queue_size=1)
        rospy.sleep(self.rest_time)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.twi_pub.publish(Twist())
        rospy.sleep(self.rest_time)

    def callback_imu(self, data):
        gyro = data.orientation
        euler = tf.transformations.euler_from_quaternion([gyro.x,gyro.y,gyro.z,gyro.w])
        self.imu_yaw = euler[2]
        end_flag = 0
        print self.imu_yaw-self.target_yaw 
        #print gyro

        if self.imu_yaw-self.target_yaw > 0.05:
            self.twist.angular.z = -0.25
        if self.imu_yaw-self.target_yaw < -0.05:
            self.twist.angular.z = 0.25
        if abs(self.imu_yaw-self.target_yaw) <= 0.05:
            self.twist.angular.z = 0.0
            end_flag = 1
        self.twi_pub.publish(self.twist)
        rospy.sleep(self.rest_time)

        if end_flag == 1:
            rospy.signal_shutdown('Ending node.')

if __name__ == '__main__':
    try:
        imu_localization()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("imu_localization killed.")

