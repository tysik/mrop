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

#include <boost/filesystem.hpp>
//#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <mtracker/Trigger.h>
#include <mtracker/Params.h>
#include <obstacle_detector/Obstacles.h>

namespace mtracker {

struct Obstacle {
  Obstacle() : x(0.0), y(0.0), r(0.0) {}
  double x, y, r;
};

class DataRecorder
{
public:
  DataRecorder();
  ~DataRecorder();

private:
  void initialize();
  void start();
  void stop();
  void addLatestData();
  void emitYamlFile();
  void emitTxtFile();

  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg);
  void refPoseCallback(const geometry_msgs::Pose2D::ConstPtr& ref_pose_msg);
  void controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg);
  void scaledControlsCallback(const geometry_msgs::Twist::ConstPtr& scaled_controls_msg);
  void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg);
  void potentialCallback(const std_msgs::Float64::ConstPtr& potential_msg);
  bool trigger(mtracker::Trigger::Request& req, mtracker::Trigger::Response& res);
  bool updateParams(mtracker::Params::Request& req, mtracker::Params::Response& res);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber pose_sub_;
  ros::Subscriber ref_pose_sub_;
  ros::Subscriber controls_sub_;
  ros::Subscriber scaled_controls_sub_;
  ros::Subscriber obstacles_sub_;
  ros::Subscriber potential_sub_;
  ros::ServiceServer trigger_srv_;
  ros::ServiceServer params_srv_;

  std::string pose_topic_;
  std::string reference_pose_topic_;
  std::string controls_topic_;
  std::string scaled_controls_topic_;
  std::string obstacles_topic_;
  std::string potential_topic_;

  geometry_msgs::Pose2D pose_;
  geometry_msgs::Pose2D ref_pose_;
  geometry_msgs::Twist controls_;
  geometry_msgs::Twist scaled_controls_;
  obstacle_detector::Obstacles obstacles_;
  double potential_;

  ros::Time start_mark_;

  std::vector<double> t_;
  std::vector<double> potential_list_;
  std::vector<geometry_msgs::Pose2D> pose_list_;
  std::vector<geometry_msgs::Pose2D> ref_pose_list_;
  std::vector<geometry_msgs::Twist> controls_list_;
  std::vector<geometry_msgs::Twist> scaled_controls_list_;
  std::vector< std::vector<Obstacle> > obstacles_list_;

  bool recording_;
  bool use_yaml_;
  bool use_txt_;
  bool record_pose_;
  bool record_reference_pose_;
  bool record_controls_;
  bool record_scaled_controls_;
  bool record_obstacles_;
  bool record_potential_;

  int loop_rate_;
};

} // namespace mtracker
