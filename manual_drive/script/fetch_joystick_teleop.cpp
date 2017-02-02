/*
 * Copyright (c) 2015, Fetch Robotics Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fetch Robotics Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Michael Ferguson

/*
 * This is still a work in progress
 * In the future, each teleop component would probably be a plugin
 */

#include <algorithm>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <topic_tools/MuxSelect.h>

double integrate(double desired, double present, double max_rate, double dt)
{
  if (desired > present)
    return std::min(desired, present + max_rate * dt);
  else
    return std::max(desired, present - max_rate * dt);
}


class TeleopComponent
{
public:
  TeleopComponent() : active_(false) {}
  virtual ~TeleopComponent() {}

  // This gets called whenever new joy message comes in
  // returns whether lower priority teleop components should be stopped
  virtual bool update(const sensor_msgs::Joy::ConstPtr& joy,
                      const sensor_msgs::JointState::ConstPtr& state) = 0;

  // This gets called at set frequency
  virtual void publish(const ros::Duration& dt) = 0;

  // Start the component. Must be idempotent.
  virtual bool start()
  {
    active_ = true;
    return active_;
  }

  // Stop the component. Must be idempotent.
  virtual bool stop()
  {
    active_ = false;
    return active_;
  }

protected:
  bool active_;
};



// Head Teleop
class HeadTeleop : public TeleopComponent
{
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client_t;

public:
  HeadTeleop(const std::string& name, ros::NodeHandle& nh) :
    last_pan_(0.0),
    last_tilt_(0.0)
  {
    ros::NodeHandle pnh(nh, name);

    // Button mapping
    pnh.param("button_deadman", deadman_, 8);
    pnh.param("axis_pan", axis_pan_, 0);
    pnh.param("axis_tilt", axis_tilt_, 3);

    // Joint limits
    pnh.param("max_vel_pan", max_vel_pan_, 1.5);
    pnh.param("max_vel_tilt", max_vel_tilt_, 1.5);
    pnh.param("max_acc_pan", max_acc_pan_, 3.0);
    pnh.param("max_acc_tilt", max_acc_tilt_, 3.0);
    pnh.param("min_pos_pan", min_pos_pan_, -1.57);
    pnh.param("max_pos_pan", max_pos_pan_, 1.57);
    pnh.param("min_pos_tilt", min_pos_tilt_, -0.76);
    pnh.param("max_pos_tilt", max_pos_tilt_, 1.45);

    // TODO: load topic from params
    head_pan_joint_ = "head_pan_joint";
    head_tilt_joint_ = "head_tilt_joint";

    std::string action_name = "head_controller/follow_joint_trajectory";
    client_.reset(new client_t(action_name, true));
    if (!client_->waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("%s may not be connected.", action_name.c_str());
    }
  }

  // This gets called whenever new joy message comes in
  virtual bool update(const sensor_msgs::Joy::ConstPtr& joy,
                      const sensor_msgs::JointState::ConstPtr& state)
  {
    bool deadman_pressed = joy->buttons[deadman_];

    if (!deadman_pressed)
    {
      stop();
      // Update joint positions
      for (size_t i = 0; i < state->name.size(); i++)
      {
        if (state->name[i] == head_pan_joint_)
          actual_pos_pan_ = state->position[i];
        if (state->name[i] == head_tilt_joint_)
          actual_pos_tilt_ = state->position[i];
      }
      return false;
    }

    desired_pan_ = joy->axes[axis_pan_] * max_vel_pan_;
    desired_tilt_ = joy->axes[axis_tilt_] * max_vel_tilt_;
    start();

    return true;
  }

  // This gets called at set frequency
  virtual void publish(const ros::Duration& dt)
  {
    if (active_)
    {
      // Fill in message (future dated with fixed time step)
      double step = 0.125;
      double pan_vel = integrate(desired_pan_, last_pan_, max_acc_pan_, step);
      double pan_travel = step * (pan_vel + last_pan_) / 2.0;
      double pan = std::max(min_pos_pan_, std::min(max_pos_pan_, actual_pos_pan_ + pan_travel));
      double tilt_vel = integrate(desired_tilt_, last_tilt_, max_acc_tilt_, step);
      double tilt_travel = step * (tilt_vel + last_tilt_) / 2.0;
      double tilt = std::max(min_pos_tilt_, std::min(max_pos_tilt_, actual_pos_tilt_ + tilt_travel));
      // Publish message
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.joint_names.push_back(head_pan_joint_);
      goal.trajectory.joint_names.push_back(head_tilt_joint_);
      trajectory_msgs::JointTrajectoryPoint p;
      p.positions.push_back(pan);
      p.positions.push_back(tilt);
      p.velocities.push_back(pan_vel);
      p.velocities.push_back(tilt_vel);
      p.time_from_start = ros::Duration(step);
      goal.trajectory.points.push_back(p);
      goal.goal_time_tolerance = ros::Duration(0.0);
      client_->sendGoal(goal);
      // Update based on actual timestep
      pan_vel = integrate(desired_pan_, last_pan_, max_acc_pan_, dt.toSec());
      pan_travel = dt.toSec() * (pan_vel + last_pan_) / 2.0;
      actual_pos_pan_ = std::max(min_pos_pan_, std::min(max_pos_pan_, actual_pos_pan_ + pan_travel));
      last_pan_ = pan_vel;
      tilt_vel = integrate(desired_tilt_, last_tilt_, max_acc_tilt_, dt.toSec());
      tilt_travel = dt.toSec() * (tilt_vel + last_tilt_) / 2.0;
      actual_pos_tilt_ = std::max(min_pos_tilt_, std::min(max_pos_tilt_, actual_pos_tilt_ + tilt_travel));
      last_tilt_ = tilt_vel;
    }
  }

  virtual bool stop()
  {
    active_ = false;
    last_pan_ = last_tilt_ = 0.0;  // reset velocities
    return active_;
  }

private:
  int deadman_, axis_pan_, axis_tilt_;
  double max_vel_pan_, max_vel_tilt_;
  double max_acc_pan_, max_acc_tilt_;
  double min_pos_pan_, max_pos_pan_, min_pos_tilt_, max_pos_tilt_;
  std::string head_pan_joint_, head_tilt_joint_;
  double actual_pos_pan_, actual_pos_tilt_;  // actual positions
  double desired_pan_, desired_tilt_;  // desired velocities
  double last_pan_, last_tilt_;
  boost::shared_ptr<client_t> client_;
};



// This pulls all the components together
class Teleop
{
  typedef boost::shared_ptr<TeleopComponent> TeleopComponentPtr;

public:
  void init(ros::NodeHandle& nh)
  {
    bool is_fetch;
    nh.param("is_fetch", is_fetch, true);

    // TODO: load these from YAML

    TeleopComponentPtr c;
    if (is_fetch)
    {
      // Torso does not override
      c.reset(new FollowTeleop("torso", nh));
      components_.push_back(c);

      // Gripper does not override
      c.reset(new GripperTeleop("gripper", nh));
      components_.push_back(c);

      // Head overrides base
      c.reset(new HeadTeleop("head", nh));
      components_.push_back(c);
    }

    // BaseTeleop goes last
    c.reset(new BaseTeleop("base", nh));
    components_.push_back(c);

    state_msg_.reset(new sensor_msgs::JointState());
    joy_sub_ = nh.subscribe("/joy", 1, &Teleop::joyCallback, this);
    state_sub_ = nh.subscribe("/joint_states", 10, &Teleop::stateCallback, this);
  }

  void publish(const ros::Duration& dt)
  {
    if (ros::Time::now() - last_update_ > ros::Duration(0.25))
    {
      // Timed out
      for (size_t c = 0; c < components_.size(); c++)
      {
        components_[c]->stop();
      }
    }
    else
    {
      for (size_t c = 0; c < components_.size(); c++)
      {
        components_[c]->publish(dt);
      }
    }
  }

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Lock mutex on state message
    boost::mutex::scoped_lock lock(state_mutex_);

    bool ok = true;
    for (size_t c = 0; c < components_.size(); c++)
    {
      if (ok)
      {
        ok &= !components_[c]->update(msg, state_msg_);
      }
      else
      {
        // supressed by a higher priority component
        components_[c]->stop();
      }
    }
    last_update_ = ros::Time::now();
  }

  void stateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    // Lock mutex on state message
    boost::mutex::scoped_lock lock(state_mutex_);

    // Update each joint based on message
    for (size_t msg_j = 0; msg_j < msg->name.size(); msg_j++)
    {
      size_t state_j;
      for (state_j = 0; state_j < state_msg_->name.size(); state_j++)
      {
        if (state_msg_->name[state_j] == msg->name[msg_j])
        {
          state_msg_->position[state_j] = msg->position[msg_j];
          state_msg_->velocity[state_j] = msg->velocity[msg_j];
          break;
        }
      }
      if (state_j == state_msg_->name.size())
      {
        // New joint
        state_msg_->name.push_back(msg->name[msg_j]);
        state_msg_->position.push_back(msg->position[msg_j]);
        state_msg_->velocity.push_back(msg->velocity[msg_j]);
      }
    }
  }

  std::vector<TeleopComponentPtr> components_;
  ros::Time last_update_;
  boost::mutex state_mutex_;
  sensor_msgs::JointStatePtr state_msg_;
  ros::Subscriber joy_sub_, state_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  ros::NodeHandle n("~");

  Teleop teleop;
  teleop.init(n);

  ros::Rate r(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    teleop.publish(ros::Duration(1/30.0));
    r.sleep();
  }

  return 0;
}
