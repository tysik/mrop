﻿/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "../include/simulator.h"

using namespace mrop;
using namespace std;

Simulator::Simulator() : nh_(""), nh_local_("~") {
  std_srvs::Empty empty;
  updateParams(empty.request, empty.response);

  params_srv_ = nh_local_.advertiseService("params", &Simulator::updateParams, this);

  ROS_INFO("Simulator [OK]");
  ros::Rate rate(p_loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    if (p_active_) {
      computeVelocity();
      computePose();
      publishAll();
    }

    rate.sleep();
  }
}

void Simulator::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

bool Simulator::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  nh_local_.param<string>("parent_frame", p_parent_frame_, "world");
  nh_local_.param<string>("child_frame", p_child_frame_, "simulator");

  nh_local_.param<bool>("active", p_active_, true);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  nh_local_.param<double>("time_constant", p_time_constant_, 0.0);
  nh_local_.param<double>("time_delay", p_time_delay_, 0.0);

  if (p_loop_rate_ > 0.0) p_sampling_time_ = 1.0 / p_loop_rate_;

  if (p_time_delay_ > p_sampling_time_) {
    lagged_pose_.assign(static_cast<int>(p_time_delay_ / p_sampling_time_), pose_);
    lagged_velocity_.assign(static_cast<int>(p_time_delay_ / p_sampling_time_), velocity_);
  }
  else  {
    lagged_pose_.assign(1, geometry_msgs::Pose2D());
    lagged_velocity_.assign(1, geometry_msgs::Twist());
  }

  if (p_active_) {
    controls_sub_ = nh_.subscribe<geometry_msgs::Twist>("controls", 5, &Simulator::controlsCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("virtual_pose", 5);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("virtual_velocity", 5);
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("virtual_pose_stamped", 5);
  }
  else {
    controls_ = geometry_msgs::Twist();
    lagged_pose_.assign(lagged_pose_.size(), pose_);
    lagged_velocity_.assign(lagged_velocity_.size(), velocity_);

    controls_sub_.shutdown();
    pose_pub_.shutdown();
    velocity_pub_.shutdown();
    pose_stamped_pub_.shutdown();
  }

  return true;
}

void Simulator::computeVelocity() {
  velocity_.linear.x  += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (controls_.linear.x - velocity_.linear.x);
  velocity_.linear.y  += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (controls_.linear.y - velocity_.linear.y);
  velocity_.angular.z += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (controls_.angular.z - velocity_.angular.z);

  lagged_velocity_.push_back(velocity_);
}

void Simulator::computePose() {
  pose_.x += p_sampling_time_ * (velocity_.linear.x * cos(pose_.theta) - velocity_.linear.y * sin(pose_.theta));
  pose_.y += p_sampling_time_ * (velocity_.linear.x * sin(pose_.theta) + velocity_.linear.y * cos(pose_.theta));
  pose_.theta += p_sampling_time_ * velocity_.angular.z;

  lagged_pose_.push_back(pose_);
}

void Simulator::publishAll() {
  geometry_msgs::Twist velocity = lagged_velocity_.front();
  geometry_msgs::Pose2D pose = lagged_pose_.front();
  geometry_msgs::PoseStamped pose_s;

  velocity_pub_.publish(velocity);
  pose_pub_.publish(pose);

  tf_.setOrigin(tf::Vector3(pose.x, pose.y, 0.0));
  tf_.setRotation(tf::createQuaternionFromYaw(pose.theta));
  tf_bc_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), p_parent_frame_, p_child_frame_));

  pose_s.header.stamp = ros::Time::now();
  pose_s.header.frame_id = p_child_frame_;
  pose_s.pose.position.x = pose.x;
  pose_s.pose.position.y = pose.y;
  pose_s.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);

  pose_stamped_pub_.publish(pose_s);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");
  Simulator s;
  return 0;
}
