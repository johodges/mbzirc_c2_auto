#!/usr/bin/env python

"""
    arm_prediction.py - Version 0.1 2015-11-05

    Use inverse kinemtatics to move the end effector to a predict position regarding current position

    Luan Cong Doan - CMS Lab
    luandoan@vt.edu

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
import sys
import moveit_commander
from geometry_msgs.msg import PoseArray, PoseStamped

class DrillPrediction:
    def __init__(self):
        #Give the node a name
        rospy.init_node("arm_prediction", anonymous=False)
        
        rospy.loginfo("Starting node arm_prediction")
        
        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander("ur5_arm")

        # Get the name of the end-effector link
        end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.0001)
        self.arm.set_goal_orientation_tolerance(0.0001)

        # Start the arm in the work pose stored in the SRDF file
        self.arm.set_named_target("work")
        self.arm.go()
        rospy.sleep(2)

        wrench_locations = rospy.wait_for_message("/initial_locations", PoseArray)
        valve_location = PoseArray()
        location = 1

        # Set the target pose
        for wrench_pose in wrench_locations.poses:
            #Reset traj(s) to None
            traj = None
            traj_nav = None
            traj_remove = None
            

            #Move the end effecor to the x - .45, y, z positon
            #Set the target pose to the wrench location in the base_link frame
            target_pose = PoseStamped()
            target_pose.header.frame_id = reference_frame
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = wrench_pose.position.x - 0.075
            target_pose.pose.position.y = wrench_pose.position.y
            target_pose.pose.position.z = wrench_pose.position.z
            target_pose.pose.orientation.x = wrench_pose.orientation.x
            target_pose.pose.orientation.y = wrench_pose.orientation.y
            target_pose.pose.orientation.z = wrench_pose.orientation.z
            target_pose.pose.orientation.w = wrench_pose.orientation.w

            # Set the start state to the current state
            self.arm.set_start_state_to_current_state()

            # Set the goal pose of the gripper to the stored pose
            self.arm.set_pose_target(target_pose, gripper_link)

            # Plan the trajectory to the goal
            traj = self.arm.plan()

            if traj is not None:
                # Execute the planned trajectory
                self.arm.execute(traj)

                # Pause for a second
                rospy.sleep(1)
                
                rospy.loginfo("Successfully moved to the predicted position " + str(location))
                
                rospy.sleep(1)
                
                rospy.loginfo("Successfully updated position of wrench " + str(location))

                # Approach the wrench
                self.arm.shift_pose_target(0, 0.075, gripper_link)
		rospy.loginfo("Successfully approached the wrench location - confirmation info needed" + str(location))

                # Plan the trajectory to the goal
                traj_drill = self.arm.plan()

                if traj_drill is not None:
                    # Execute the planned trajectory
                    self.arm.execute(traj_nav)
                    
                    # Pause for a second
                    rospy.sleep(1)

                    rospy.loginfo("Successfully picked wrench " + str(location))
                    location += 1
                    
                    self.arm.shift_pose_target(0, -0.075, gripper_link)
                    
		## moving arm from wrench location to valve location - sensor data needed - need extra work!!!
		## How to get the correct wrench size -> arm navigation trajectory
		## Next 10 following lines should be corrected base on technique

                    # Plan the trajectory to the goal
                    traj_remove = self.arm.plan()
                    
                    if traj_remove is not None:
                        # Execute the planned trajectory
                        self.arm.execute(traj_remove)
                        
                        # Pause for a second
                        rospy.sleep(1)
                        
                    else:
                        continue
            
                else:
                    rospy.loginfo("Unable to reach the wrench position " + str(location))
                    valve.poses.append(target_pose)
                    location += 1
                    continue                   
                
        # Finish the ur5_arm in the stow pose stored in the SRDF file
        self.arm.set_named_target("stow")
        self.arm.go()
        rospy.sleep(2)
        
    def cleanup(self):
        rospy.loginfo("Stopping the robot")
    
        # Stop any current arm movement
        self.arm.stop()
        
        # Move the arm to the stow position
        self.arm.set_named_target("stow")
        self.arm.go()
        
        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
  try:
    ArmPrediction()
  except KeyboardInterrupt:
      print "Shutting down ArmPrediction node."
