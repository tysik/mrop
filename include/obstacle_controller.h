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
 * Author: Mateusz Przybyla and Wojciech Kowalczyk
 */

#pragma once

#define ARMA_DONT_USE_CXX11

#include <armadillo>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <mtracker/Trigger.h>
#include <mtracker/Params.h>
#include <obstacle_detector/Obstacles.h>

namespace mtracker
{

struct Obstacle {
  Obstacle() : x(0.0), y(0.0), r(0.0) {}
  double x, y, r;
};

class ObstacleController
{
public:
  ObstacleController();

private:
  void initialize();
  void computeControls();

  double getBetaWorld();          // World beta function
  double getBeta_i(const Obstacle& o);   // Obstacles beta functions
  double getBeta();               // Total beta function (product of all betas)

  arma::vec getGradBetaWorld();         // Gradient of world beta function
  arma::vec getGradBeta_i(const Obstacle& o);  // Gradient of obstacles beta functions
  arma::vec getGradBeta();              // Gradient of total beta function

  double getV();            // Navigation function
  arma::vec getGradV();     // Gradient of navigation function

  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg);
  void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg);
  bool trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res);
  bool updateParams(mtracker::Params::Request &req, mtracker::Params::Response &res);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber pose_sub_;
  ros::Subscriber obstacles_sub_;
  ros::Publisher controls_pub_;
  ros::Publisher potential_pub_;
  ros::ServiceServer trigger_srv_;
  ros::ServiceServer params_srv_;

  std::string pose_topic_;
  std::string controls_topic_;
  std::string obstacles_topic_;
  std::string potential_topic_;

  geometry_msgs::Twist controls_;
  geometry_msgs::Pose2D pose_;

  Obstacle world_;                   // Negative obstacle containing world
  std::vector<Obstacle> obstacles_;  // List of obstacles

  double a_, b_dash_, k_w_, epsilon_, kappa_;   // Constant parameters

  int loop_rate_;
  bool obstacle_controller_active_;
};

} // namespace mtracker
