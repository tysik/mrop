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

#include "../include/manual_controller.h"

using namespace mrop;

ManualController::ManualController() : nh_(""), nh_local_("~") {
  {
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;

    updateParams(req, res);
    p_manual_controller_active_ = !p_manual_controller_active_;
    trigger(req, res);
  }

  trigger_srv_ = nh_.advertiseService("manual_controller_trigger_srv", &ManualController::trigger, this);
  params_srv_ = nh_.advertiseService("manual_controller_params_srv", &ManualController::updateParams, this);

  ROS_INFO("Manual controller [OK]");
  ros::spin();
}

void ManualController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
  if (p_use_joy_) {
    controls_.linear.x = p_k_v_ * joy_msg->axes[1];
    controls_.linear.y = p_k_v_ * joy_msg->axes[0];
    controls_.angular.z = p_k_w_ * joy_msg->axes[2];
    controls_pub_.publish(controls_);
  }
}

void ManualController::keysCallback(const geometry_msgs::Twist::ConstPtr& keys_msg) {
  if (p_use_keys_) {
    controls_.linear.x = p_k_v_ * keys_msg->linear.x;
    controls_.linear.y = p_k_v_ * keys_msg->linear.y;
    controls_.angular.z = p_k_w_ * keys_msg->angular.z;
    controls_pub_.publish(controls_);
  }
}

bool ManualController::trigger(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  p_manual_controller_active_ = !p_manual_controller_active_;

  if (p_manual_controller_active_) {
    controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 5);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 5, &ManualController::joyCallback, this);
    keys_sub_ = nh_.subscribe<geometry_msgs::Twist>("keys", 5, &ManualController::keysCallback, this);
  }
  else {
    controls_.linear.x = 0.0;
    controls_.angular.z = 0.0;
    controls_pub_.publish(controls_);

    controls_pub_.shutdown();
    joy_sub_.shutdown();
    keys_sub_.shutdown();
  }

  return true;
}

bool ManualController::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  nh_local_.param<bool>("manual_controller_active", p_manual_controller_active_, true);
  nh_local_.param<bool>("use_joy", p_use_joy_, true);
  nh_local_.param<bool>("use_keys", p_use_keys_, false);

  nh_local_.param<double>("linear_gain", p_k_v_, 0.25);
  nh_local_.param<double>("angular_gain", p_k_w_, 1.0);

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_controller");
  ManualController mc;
  return 0;
}

