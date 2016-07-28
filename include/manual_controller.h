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

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

namespace mrop
{

class ManualController
{
public:
  ManualController();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void keysCallback(const geometry_msgs::Twist::ConstPtr& keys_msg);
  bool trigger(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber joy_sub_;
  ros::Subscriber keys_sub_;

  ros::Publisher controls_pub_;

  ros::ServiceServer trigger_srv_;
  ros::ServiceServer params_srv_;

  geometry_msgs::Twist controls_;

  // Parameters
  bool p_manual_controller_active_;
  bool p_use_joy_;
  bool p_use_keys_;

  double p_k_v_;  // Linear velocity gain
  double p_k_w_;  // Angular velocity gain
};

} // namespace mrop
