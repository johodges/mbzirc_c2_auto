/*********************************************************************
backitup_recovery.cpp - Version 1.0 2017-01-02
Author: Jonathan Hodges
Based on: rotate_recovery.cpp by Eitan Marder-Eppstein

This software is an alternate recovery scheme for the move_base
package in ROS. The recovery behavior moves the robot backward and
rotates clockwise a fixed amount.

Parameters:
    vel: linear velocity to move
    dist: linear distance to move
    ang_vel: angular velocity to move
    ang_dist: angular distance to move

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html

*********************************************************************/

#include <backitup_recovery/backitup_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(backitup_recovery, BackItUpRecovery, backitup_recovery::BackItUpRecovery, nav_core::RecoveryBehavior)

namespace backitup_recovery {
BackItUpRecovery::BackItUpRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void BackItUpRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    //we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
    blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

BackItUpRecovery::~BackItUpRecovery(){
  delete world_model_;
}

void BackItUpRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the BackItUpRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("BackItUp recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("joy_teleop/cmd_vel", 1);

  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->getRobotPose(global_pose);

    double vel = 0.25;
    double dist = 0.5;
    double move_time = dist/vel;
    double rest_time = 0.1;

    ROS_WARN("Moving backward: %.1f m.", dist);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = -1*vel;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    double time = 0;
    while(time < move_time){
        vel_pub.publish(cmd_vel);
        time = time + rest_time;
        ros::Duration(rest_time).sleep();
    }

    double ang_vel = 0.25;
    double ang_dist = 0.25;
    move_time = ang_dist/ang_vel;

    ROS_WARN("Rotating clockwise: %.1f rad.", ang_dist);
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = ang_vel;
    time = 0;
    while(time < move_time){
        vel_pub.publish(cmd_vel);
        time = time + rest_time;
        ros::Duration(rest_time).sleep();
    }

    return;

  }
};
