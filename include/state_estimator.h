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
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <mtracker/Trigger.h>
#include <mtracker/Params.h>

namespace mtracker
{

class StateEstimator
{
public:
  StateEstimator();

private:
  void initialize();
  void estimateState();
  void publishAll();

  void controlsCallback(const geometry_msgs::Twist::ConstPtr& scaled_controls_msg);
  void odomPoseCallback(const geometry_msgs::Pose2D::ConstPtr& odom_pose_msg);
  void optitrackPoseCallback(const geometry_msgs::Pose2D::ConstPtr& opti_pose_msg);
  bool trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res);
  bool updateParams(mtracker::Params::Request &req, mtracker::Params::Response &res);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber scaled_controls_sub_;
  ros::Subscriber odom_pose_sub_;
  ros::Subscriber optitrack_pose_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher pose_stamped_pub_;
  ros::Publisher velocity_pub_;
  ros::ServiceServer trigger_srv_;
  ros::ServiceServer params_srv_;

  std::string scaled_controls_topic_;
  std::string odom_pose_topic_;
  std::string optitrack_pose_topic_;
  std::string velocity_topic_;
  std::string pose_topic_;

  tf::TransformBroadcaster pose_bc_;
  tf::Transform pose_tf_;

  std::string world_frame_;
  std::string child_frame_;

  geometry_msgs::Twist scaled_controls_;
  geometry_msgs::Pose2D odom_pose_;
  geometry_msgs::Pose2D opti_pose_;
  geometry_msgs::Pose2D pose_;
  geometry_msgs::Twist velocity_;

  int loop_rate_;
  bool state_estimator_active_;
};

} // namespace mtracker
