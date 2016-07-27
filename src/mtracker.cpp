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

#include "../include/mtracker.h"

using namespace mtracker;

MTracker::MTracker() : nh_(""), nh_local_("~"), ROBOT_BASE(0.145), WHEEL_RADIUS(0.025), mtracker_active_(false) {
  initialize();

  com_ = new Serial("/dev/ttyUSB0");
  com_->open(921600);

  if (com_->isOpen()) {
    com_->setMode(MODE_SET_ODOMETRY | MODE_MOTORS_ON);
    com_->setPose(0.0, 0.0, 0.0);
    com_->setVelocities(0.0, 0.0);
    com_->writeFrame();

    ROS_INFO("MTracker [OK]");
  }
  else {
    ROS_INFO("Could not open COM port.");
    nh_.shutdown();
  }

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    if (mtracker_active_) {
      transferData();

      pose_pub_.publish(pose_);
      velocity_pub_.publish(velocity_);
    }

    rate.sleep();
  }
}

MTracker::~MTracker() {
  com_->setVelocities(0.0, 0.0);
  com_->setMode(MODE_MOTORS_OFF);
  com_->writeFrame();

  delete com_;
}

void MTracker::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  if (!nh_.getParam("scaled_controls_topic", scaled_controls_topic_))
    scaled_controls_topic_ = "scaled_controls";

  if (!nh_.getParam("odom_pose_topic", odom_pose_topic_))
    odom_pose_topic_ = "odom_pose";

  if (!nh_.getParam("odom_velocity_topic", odom_velocity_topic_))
    odom_velocity_topic_ = "odom_velocity";

  trigger_srv_ = nh_.advertiseService("mtracker_trigger_srv", &MTracker::trigger, this);
  params_srv_ = nh_.advertiseService("mtracker_params_srv", &MTracker::updateParams, this);
}

void MTracker::transferData() {
  double w_l = (controls_.linear.x - ROBOT_BASE * controls_.angular.z / 2.0) / WHEEL_RADIUS;
  double w_r = (controls_.linear.x + ROBOT_BASE * controls_.angular.z / 2.0) / WHEEL_RADIUS;

  com_->setVelocities(w_l, w_r);
  com_->writeFrame();
  com_->readFrame();

  pose_ = com_->getPose();

  w_l = com_->getVelocities().angular.x;
  w_r = com_->getVelocities().angular.y;

  velocity_.linear.x  = (w_r + w_l) * WHEEL_RADIUS / 2.0;
  velocity_.angular.z = (w_r - w_l) * WHEEL_RADIUS / ROBOT_BASE;
}

void MTracker::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

bool MTracker::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  mtracker_active_ = req.activate;

  if (req.activate) {
    controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(scaled_controls_topic_, 5, &MTracker::controlsCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(odom_pose_topic_, 5);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(odom_velocity_topic_, 5);
  }
  else {
    com_->setVelocities(0.0, 0.0);
    com_->writeFrame();
    com_->readFrame();

    controls_sub_.shutdown();
    pose_pub_.shutdown();
    velocity_pub_.shutdown();
  }

  return true;
}

bool MTracker::updateParams(mtracker::Params::Request &req, mtracker::Params::Response &res) {
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mtracker");
  MTracker mt;
  return 0;
}
