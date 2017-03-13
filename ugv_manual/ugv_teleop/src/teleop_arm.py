#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import cv2
from robot_mv_cmds import *

class arm_controller:
    def __init__(self,x,y,z):
        self.resolution = 0.1
        self.x = x
        self.y = y
        self.z = z
        self.subscriber = rospy.Subscriber("/joy", Joy, self.callback)


    def callback(self, data):
        success_flag=data.buttons[7]
        failed_flag=data.buttons[6]
        #arm control
        resolution_increase=data.buttons[5]
        resolution_decrease=data.buttons[4]

        arm_y_control = data.axes[6]
        arm_z_control = data.axes[7]
        arm_x_control_increase = data.buttons[3]
        arm_x_control_decrease = data.buttons[0]
        if(success_flag==1 and failed_flag==0):
            rospy.set_param('smach_state', 'success')
            print 'success!'
        elif(success_flag==0 and failed_flag==1):
            rospy.set_param('smach_state', 'failed')
            print 'failed!'

        self.control_arm(resolution_increase, resolution_decrease, arm_y_control,arm_z_control, arm_x_control_increase, arm_x_control_decrease)

    def control_arm(self, resolution_increase, resolution_decrease, arm_y_control,arm_z_control, arm_x_control_increase, arm_x_control_decrease):
        print 2*self.resolution

        if(resolution_increase>0):
            self.resolution = 2 * self.resolution
        elif(resolution_decrease>0):
            self.resolution = 0.5 * self.resolution
        # y control
        elif(arm_y_control>0):
            self.y = self.y + self.resolution
        elif(arm_y_control<0):
            self.y = self.y - self.resolution
        # z control
        elif(arm_z_control>0):
            self.z = self.z + self.resolution
        elif(arm_z_control<0):
            self.z = self.z - self.resolution
        # x control
        elif(arm_x_control_increase==1):
            self.x = self.x + self.resolution
        elif(arm_x_control_decrease==1):
            self.x = self.x - self.resolution

        print 'Current state'
        print [self.x, self.y, self.z]
        print 'Current resolution'
        print self.resolution

        # Move the arm. Commented during the development
        moveArmTwist(self.x,self.y,self.z)

def listener():
    rospy.init_node('teleop', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    #rospy.Subscriber("/joy_teleop/joy", Joy, callback)
    rospy.spin()



if __name__ == '__main__':
    rospy.init_node('teleop', anonymous=True)
    x_init = rospy.get_param('x', 3)
    y_init = rospy.get_param('y', 2)
    z_init = rospy.get_param('z', 1)
    #ac=arm_controller(3,2,1)
    ac=arm_controller(x_init,y_init,z_init)
    rospy.spin()


