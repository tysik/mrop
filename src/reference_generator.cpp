/*
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

#include "../include/reference_generator.h"

using namespace mrop;
using namespace std;

ReferenceGenerator::ReferenceGenerator() : nh_(""), nh_local_("~") {
  {
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;

    updateParams(req, res);
    p_reference_generator_active_ = !p_reference_generator_active_;
    trigger(req, res);
  }

  trigger_srv_ = nh_.advertiseService("reference_generator_trigger_srv", &ReferenceGenerator::trigger, this);
  params_srv_ = nh_.advertiseService("reference_generator_params_srv", &ReferenceGenerator::updateParams, this);

  ROS_INFO("Reference generator [OK]");
  ros::Rate rate(p_loop_rate_);
  ros::Time time_stamp = ros::Time::now();

  while (nh_.ok()) {
    ros::spinOnce();

    if (p_reference_generator_active_) {
      double dt = (ros::Time::now() - time_stamp).toSec();
      time_stamp = ros::Time::now();

      if (!p_paused_)
        update(dt);

      publishAll();
    }

    rate.sleep();
  }
}

ReferenceGenerator::~ReferenceGenerator() {
  delete trajectory_;
}

bool ReferenceGenerator::trigger(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  p_reference_generator_active_ = !p_reference_generator_active_;

  if (p_reference_generator_active_) {
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("reference_pose", 5);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("reference_velocity", 5);
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference_pose_stamped", 5);
  }
  else {
    p_paused_ = true;
    p_stopped_ = true;

    time_ = 0.0;
    update(0.0);

    pose_pub_.shutdown();
    velocity_pub_.shutdown();
    pose_stamped_pub_.shutdown();
  }

  return true;
}

bool ReferenceGenerator::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  nh_local_.param<bool>("reference_generator_active", p_reference_generator_active_, true);
  nh_local_.param<bool>("trajectory_paused", p_paused_, false);
  nh_local_.param<bool>("trajectory_stopped", p_stopped_, false);

  nh_local_.param<string>("parent_frame", p_parent_frame_, "world");
  nh_local_.param<string>("child_frame", p_child_frame_, "reference");

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  nh_local_.param<int>("trajectory_type", p_trajectory_type_, 0);

  nh_local_.param<double>("initial_x", p_x_0_, 0.0);
  nh_local_.param<double>("initial_y", p_y_0_, 0.0);
  nh_local_.param<double>("initial_theta", p_theta_0_, 0.0);
  nh_local_.param<double>("linear_velocity", p_v_, 0.1);
  nh_local_.param<double>("harmonic_period", p_T_, 5.0);
  nh_local_.param<double>("harmonic_radius_x", p_r_x_, 1.0);
  nh_local_.param<double>("harmonic_radius_y", p_r_y_, 1.0);
  nh_local_.param<double>("harmonic_multiplier_x", p_n_x_, 1.0);
  nh_local_.param<double>("harmonic_multiplier_y", p_n_y_, 1.0);

  switch (p_trajectory_type_) {
    case 0:
      trajectory_ = new Trajectory(p_x_0_, p_y_0_, p_theta_0_); break;
    case 1:
      trajectory_ = new LinearTrajectory(p_x_0_, p_y_0_, p_theta_0_, p_v_); break;
    case 2:
      trajectory_ = new HarmonicTrajectory(p_x_0_, p_y_0_, p_T_, p_r_x_, p_r_y_, p_n_x_, p_n_y_); break;
    case 3:
      trajectory_ = new LemniscateTrajectory(p_x_0_, p_y_0_, p_T_, p_r_x_, p_r_y_, p_n_x_, p_n_y_); break;
    default:
      trajectory_ = new Trajectory(0.0, 0.0, 0.0); break;
  }

  if (!p_paused_ && !p_stopped_)
    start();
  else if (p_paused_ && !p_stopped_)
    pause();
  else
    stop();

  return true;
}

void ReferenceGenerator::start() {
  p_paused_ = false;
}

void ReferenceGenerator::stop() {
  p_paused_ = true;
  time_ = 0.0;
  update(0.0);
}

void ReferenceGenerator::pause() {
  p_paused_ = true;
}

void ReferenceGenerator::update(double dt) {
  double prev_theta = pose_.theta;
  double prev_theta_aux = atan2(sin(pose_.theta), cos(pose_.theta));

  time_ += dt;
  pose_ = trajectory_->calculatePose(time_);
  velocity_ = trajectory_->calculateVelocity(time_);

  double new_theta_aux = atan2(sin(pose_.theta), cos(pose_.theta));
  double theta_diff = new_theta_aux - prev_theta_aux;

  if (theta_diff < -M_PI)
    pose_.theta = prev_theta + theta_diff + 2.0 * M_PI;
  else if (theta_diff > M_PI)
    pose_.theta = prev_theta + theta_diff - 2.0 * M_PI;
  else
    pose_.theta = prev_theta + theta_diff;
}

void ReferenceGenerator::publishAll() {
  pose_pub_.publish(pose_);
  velocity_pub_.publish(velocity_);

  tf_.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
  tf_.setRotation(tf::createQuaternionFromYaw(pose_.theta));
  tf_br_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), p_parent_frame_, p_child_frame_));

  geometry_msgs::PoseStamped pose_s;

  pose_s.header.stamp = ros::Time::now();
  pose_s.header.frame_id = p_child_frame_;

  pose_s.pose.position.x = pose_.x;
  pose_s.pose.position.y = pose_.y;
  pose_s.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.theta);

  pose_stamped_pub_.publish(pose_s);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "reference_generator");
  ReferenceGenerator rg;
  return 0;
}
