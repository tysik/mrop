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

using namespace mrop;

AutomaticController::AutomaticController() : nh_(""), nh_local_("~") {
  {
    std_srvs::Empty empt;
    std_srvs::Trigger trig;

    updateParams(empt.request, empt.response);
    p_automatic_controller_active_ = !p_automatic_controller_active_; // Trigger switches the boolean
    trigger(trig.request, trig.response);
  }

  trigger_srv_ = nh_local_.advertiseService("trigger_srv", &AutomaticController::trigger, this);
  params_srv_  = nh_local_.advertiseService("params_srv", &AutomaticController::updateParams, this);

  ROS_INFO("Automatic controller [OK]");
  ros::Rate rate(p_loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    if (p_automatic_controller_active_) {
      computeControls();
      publishAll();

      ROS_INFO_STREAM(p_k_p_);
    }

    rate.sleep();
  }
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

bool AutomaticController::trigger(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  p_automatic_controller_active_ = !p_automatic_controller_active_;
  res.success = p_automatic_controller_active_;
  nh_local_.setParam("automatic_controller_active", p_automatic_controller_active_);

  if (p_automatic_controller_active_) {
    pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("pose", 5, &AutomaticController::poseCallback, this);
    velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("velocity", 5, &AutomaticController::velocityCallback, this);
    ref_pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("reference_pose", 5, &AutomaticController::refPoseCallback, this);
    ref_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("reference_velocity", 5, &AutomaticController::refVelocityCallback, this);
    controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 5);
  }
  else {
    pose_sub_.shutdown();
    velocity_sub_.shutdown();
    ref_pose_sub_.shutdown();
    ref_velocity_sub_.shutdown();
    controls_pub_.shutdown();
  }

  return true;
}

bool AutomaticController::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  nh_local_.param<bool>("automatic_controller_active", p_automatic_controller_active_, true);
  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  nh_local_.param<double>("gain", p_k_p_, 1.0);

  return true;
}

void AutomaticController::computeControls() {
  /*
   * The following variables are described in the world coordinate system
   */
  double x = pose_.x;
  double y = pose_.y;
  double theta = pose_.theta;

  double v_x = velocity_.linear.x;
  double v_y = velocity_.linear.y;
  double w_z = velocity_.angular.z;

  double x_d = ref_pose_.x;
  double y_d = ref_pose_.y;
  double theta_d = ref_pose_.theta;

  double v_x_d = ref_velocity_.linear.x;
  double v_y_d = ref_velocity_.linear.y;
  double w_z_d = ref_velocity_.angular.z;

  /*
   * The control signals must be described in the base coordinate system
   */
  double u = 0.0; // Forward linear velocity
  double v = 0.0; // Sideways linear velocity
  double w = 0.0; // Angular velocity

  /*
   * HERE PUT THE CONTROLLER CODE
   */

  controls_.linear.x = u;
  controls_.linear.y = v;
  controls_.angular.z = w;
}

void AutomaticController::publishAll() {
  controls_pub_.publish(controls_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "automatic_controller");
  AutomaticController ac;
  return 0;
}
