"""Commands for directly moving the arm and UGV

Author: Tom Ehrenzeller, VT
Modifications: Alan Lattimer, Jensen Hughes

Create Date: 12-13-2016
Modified Date: 12-14-2016

--------------------------------------------------------------------------
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.5

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details at:

http://www.gnu.org/licenses/gpl.html

"""

import rospy
from geometry_msgs.msg import Twist

def moveArmTwist(x, y, z):
    """Move arm to new location

    Function to move arm end effector to an x,y,z position

    Args:
        x,y,z: coordinate position of where to move the arm

    Returns:
        move_state: success or failure of moving the arm

    Publishers:
        goal_pub: publishes a goal twist to /move_arm/goal
    """
    sleep_time = 0.1
    # Prepare system for a new goal
    move_state = 'SendGoal'
    rospy.set_param('move_arm_status',move_state)
    try:
        physical_robot = rospy.get_param('physical_robot')
    except:
        physical_robot = 'false'
    # Set up publisher
    goal_pub = rospy.Publisher("/move_arm/goal",Twist, queue_size=1)

    # Define the goal position
    tw = Twist()
    tw.linear.x = x
    tw.linear.y = y
    tw.linear.z = z
    """
    if physical_robot:
        try:
            sawyer = rospy.get_param('sawyer')
        except:
            sawyer = 'false'
        if sawyer:
            tw.angular.x = 1.57
            tw.angular.y = 1.57
            tw.angular.z = 1.57
        if not sawyer:
            tw.angular.x = 0
            tw.angular.y = 0
            tw.angular.z = 0#-1.57
    """
    """
    print "Physical?"
    print physical_robot
    print "Twist:", tw
    """
    rospy.set_param('move_arm_status','Sent')
    while move_state != 'success':
    #for i in range(50):
        #for k in range(5):
            # Publish the goal five times
        goal_pub.publish(tw)
        rospy.sleep(sleep_time)
        move_state = rospy.get_param('move_arm_status')
        if move_state == 'success':
            break
        else:
            rospy.logdebug("Current move arm status is %s",move_state)

    return move_state


def moveUGVvel(vel, dist_to_move, move_type='linear'):
    """Move the UGV

    Moves the UGV with either a linear or angular velocity the distance given

    Args:
        vel: velocity to move the UGV
        dist_to_move: move distance in meters for linear and radians for angular
        move_type: type of move

    Publishers:
        vel_pub: publishes a velocity twist message to /joy_teleop/cmd_vel
    """
    sleep_time = 0.1
    time_to_move = abs(dist_to_move/vel)

    vel_twist = Twist()
    if move_type == 'linear':
        vel_twist.linear.x = vel
    else:
        vel_twist.angular.z = vel

    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    ct_move = 0
    while ct_move*sleep_time < time_to_move:
        vel_pub.publish(vel_twist)
        ct_move = ct_move+1
        rospy.sleep(sleep_time)
    vel_twist = Twist()
    vel_pub.publish(vel_twist)
    rospy.sleep(sleep_time)
