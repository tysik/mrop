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

#include "../include/state_estimator.h"

using namespace mrop;
using namespace std;

StateEstimator::StateEstimator() : nh_(""), nh_local_("~") {
  {
    std_srvs::Empty empt;
    std_srvs::Trigger trig;

    updateParams(empt.request, empt.response);
    p_state_estimator_active_ = !p_state_estimator_active_;
    trigger(trig.request, trig.response);
  }

  trigger_srv_ = nh_.advertiseService("state_estimator_trigger_srv", &StateEstimator::trigger, this);
  params_srv_ = nh_.advertiseService("state_estimator_params_srv", &StateEstimator::updateParams, this);

  ROS_INFO("State estimator [OK]");
  ros::Rate rate(p_loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    if (p_state_estimator_active_) {
      estimateState();
      publishAll();
    }

    rate.sleep();
  }
}

void StateEstimator::controlsCallback(const geometry_msgs::Twist::ConstPtr& scaled_controls_msg) {
  scaled_controls_ = *scaled_controls_msg;
}

void StateEstimator::odomPoseCallback(const geometry_msgs::Pose2D::ConstPtr& odom_pose_msg) {
  odom_pose_ = *odom_pose_msg;
}

void StateEstimator::optitrackPoseCallback(const geometry_msgs::Pose2D::ConstPtr& opti_pose_msg) {
  double prev_theta = opti_pose_.theta;
  double prev_theta_aux = atan2(sin(opti_pose_.theta), cos(opti_pose_.theta));

  opti_pose_.x =  opti_pose_msg->y;
  opti_pose_.y = -opti_pose_msg->x;
  opti_pose_.theta = opti_pose_msg->theta;

  double new_theta_aux = atan2(sin(opti_pose_.theta), cos(opti_pose_.theta));
  double theta_diff = new_theta_aux - prev_theta_aux;

  if (theta_diff < -M_PI)
    opti_pose_.theta = prev_theta + theta_diff + 2.0 * M_PI;
  else if (theta_diff > M_PI)
    opti_pose_.theta = prev_theta + theta_diff - 2.0 * M_PI;
  else
    opti_pose_.theta = prev_theta + theta_diff;
}

bool StateEstimator::trigger(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  p_state_estimator_active_ = !p_state_estimator_active_;

  if (p_state_estimator_active_) {
    scaled_controls_sub_ = nh_.subscribe<geometry_msgs::Twist>("scaled_controls", 10, &StateEstimator::controlsCallback, this);
    odom_pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("odom_pose", 10, &StateEstimator::odomPoseCallback, this);
    optitrack_pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("optitrack_pose", 10, &StateEstimator::optitrackPoseCallback, this);

    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("pose", 10);
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_stamped", 10);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("velocity", 10);
  }
  else {
    scaled_controls_sub_.shutdown();
    odom_pose_sub_.shutdown();
    optitrack_pose_sub_.shutdown();
    pose_pub_.shutdown();
    pose_stamped_pub_.shutdown();
    velocity_pub_.shutdown();
  }

  res.success = p_state_estimator_active_;

  return true;
}

bool StateEstimator::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  nh_local_.param<string>("parent_frame", p_parent_frame_, "world");
  nh_local_.param<string>("child_frame_", p_child_frame_, "robot");

  nh_local_.param<bool>("state_estimator_active", p_state_estimator_active_, true);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);

  return true;
}

void StateEstimator::estimateState() {
  pose_ = opti_pose_;
  velocity_ = scaled_controls_;
}

void StateEstimator::publishAll() {
  pose_pub_.publish(pose_);
  velocity_pub_.publish(velocity_);

  geometry_msgs::PoseStamped pose_s;

  pose_s.header.stamp = ros::Time::now();
  pose_s.header.frame_id = p_child_frame_;
  pose_s.pose.position.x = pose_.x;
  pose_s.pose.position.y = pose_.y;
  pose_s.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.theta);

  pose_stamped_pub_.publish(pose_s);

  pose_tf_.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
  pose_tf_.setRotation(tf::createQuaternionFromYaw(pose_.theta));
  pose_bc_.sendTransform(tf::StampedTransform(pose_tf_, ros::Time::now(), p_parent_frame_, p_child_frame_));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_estimator");
  StateEstimator se;
  return 0;
}
