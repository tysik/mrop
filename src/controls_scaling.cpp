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

#include "../include/controls_scaling.h"

using namespace mtracker;

ControlsScaling::ControlsScaling() : nh_(""), nh_local_("~"), ROBOT_BASE(0.145), WHEEL_RADIUS(0.025), controls_scaling_active_(false) {
  initialize();

  ROS_INFO("MTracker controls scaling [OK]");

  ros::spin();
}

void ControlsScaling::initialize() {
  if (!nh_.getParam("controls_topic", controls_topic_))
    controls_topic_ = "controls";

  if (!nh_.getParam("scaled_controls_topic", scaled_controls_topic_))
    scaled_controls_topic_ = "scaled_controls";

  if (!nh_local_.getParam("max_wheel_rate", max_wheel_rate_))
    max_wheel_rate_ = 15.0;

  controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(controls_topic_, 5, &ControlsScaling::controlsCallback, this);
  controls_pub_ = nh_.advertise<geometry_msgs::Twist>(scaled_controls_topic_, 5);
  trigger_srv_ = nh_.advertiseService("controls_scaling_trigger_srv", &ControlsScaling::trigger, this);
  params_srv_ = nh_.advertiseService("controls_scaling_params_srv", &ControlsScaling::updateParams, this);
}

void ControlsScaling::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  geometry_msgs::Twist scaled_controls;

  if (controls_scaling_active_) {
    double w_l = (controls_msg->linear.x - ROBOT_BASE * controls_msg->angular.z / 2.0) / WHEEL_RADIUS;
    double w_r = (controls_msg->linear.x + ROBOT_BASE * controls_msg->angular.z / 2.0) / WHEEL_RADIUS;

    double s = 1.0; // Scaling factor

    if (abs(w_l) >= abs(w_r) && abs(w_l) > max_wheel_rate_)
      s = max_wheel_rate_ / abs(w_l);
    else if (abs(w_l) <= abs(w_r) && abs(w_r) > max_wheel_rate_)
      s = max_wheel_rate_ / abs(w_r);

    w_l *= s;
    w_r *= s;

    scaled_controls.linear.x  = (w_r + w_l) * WHEEL_RADIUS / 2.0;
    scaled_controls.angular.z = (w_r - w_l) * WHEEL_RADIUS / ROBOT_BASE;
  }
  else
    scaled_controls = *controls_msg;

  controls_pub_.publish(scaled_controls);
}

bool ControlsScaling::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  controls_scaling_active_ = req.activate;
  return true;
}

bool ControlsScaling::updateParams(mtracker::Params::Request &req, mtracker::Params::Response &res) {
  if (req.params.size() >= 1) {
    max_wheel_rate_ = req.params[0];
    return true;
  }
  else
    return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "controls_scaling");
  ControlsScaling cs;
  return 0;
}

