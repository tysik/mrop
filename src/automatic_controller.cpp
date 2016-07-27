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

#include "../include/automatic_controller.h"

using namespace mtracker;

AutomaticController::AutomaticController() : nh_(""), nh_local_("~"), automatic_controller_active_(false) {
  initialize();

  ROS_INFO("Automatic controller [OK]");

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    if (automatic_controller_active_) {
      computeControls();
      publishAll();
    }

    rate.sleep();
  }
}

void AutomaticController::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  if (!nh_.getParam("pose_topic", pose_topic_))
    pose_topic_ = "pose";

  if (!nh_.getParam("velocity_topic", velocity_topic_))
    velocity_topic_ = "velocity";

  if (!nh_.getParam("reference_pose_topic", reference_pose_topic_))
    reference_pose_topic_ = "reference_pose";

  if (!nh_.getParam("reference_velocity_topic", reference_velocity_topic_))
    reference_velocity_topic_ = "reference_velocity";

  if (!nh_.getParam("controls_topic", controls_topic_))
    controls_topic_ = "controls";

  trigger_srv_ = nh_.advertiseService("automatic_controller_trigger_srv", &AutomaticController::trigger, this);
  params_srv_ = nh_.advertiseService("automatic_controller_params_srv", &AutomaticController::updateParams, this);
}

void AutomaticController::computeControls() {
  double x = pose_.x;
  double y = pose_.y;
  double theta = pose_.theta;

  double x_d = ref_pose_.x;
  double y_d = ref_pose_.y;
  double theta_d = ref_pose_.theta;

  double v = 0.0; // Linear velocity
  double w = 0.0; // Angular velocity

  /*
   * HERE PUT THE CODE
   */

  controls_.linear.x = v;
  controls_.angular.z = w;
}

void AutomaticController::publishAll() {
  controls_pub_.publish(controls_);
}

void AutomaticController::poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
  pose_ = *pose_msg;
}

void AutomaticController::velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg) {
  velocity_ = *velocity_msg;
}

void AutomaticController::refPoseCallback(const geometry_msgs::Pose2D::ConstPtr& ref_pose_msg) {
  ref_pose_ = *ref_pose_msg;
}

void AutomaticController::refVelocityCallback(const geometry_msgs::Twist::ConstPtr& ref_velocity_msg) {
  ref_velocity_ = *ref_velocity_msg;
}

bool AutomaticController::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  automatic_controller_active_ = req.activate;

  if (req.activate) {
    pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(pose_topic_, 5, &AutomaticController::poseCallback, this);
    velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>(velocity_topic_, 5, &AutomaticController::velocityCallback, this);
    ref_pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(reference_pose_topic_, 5, &AutomaticController::refPoseCallback, this);
    ref_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>(reference_velocity_topic_, 5, &AutomaticController::refVelocityCallback, this);
    controls_pub_ = nh_.advertise<geometry_msgs::Twist>(controls_topic_, 5);
  }
  else {
    controls_.linear.x = 0.0;
    controls_.angular.z = 0.0;
    controls_pub_.publish(controls_);

    pose_sub_.shutdown();
    velocity_sub_.shutdown();
    ref_pose_sub_.shutdown();
    ref_velocity_sub_.shutdown();
    controls_pub_.shutdown();
  }

  return true;
}

bool AutomaticController::updateParams(mtracker::Params::Request &req, mtracker::Params::Response &res) {
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "automatic_controller");
  AutomaticController ac;
  return 0;
}
