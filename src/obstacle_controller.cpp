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

#include "../include/obstacle_controller.h"

using namespace mtracker;
using namespace arma;

ObstacleController::ObstacleController() : nh_(""), nh_local_("~"), obstacle_controller_active_(false) {
  initialize();

  ROS_INFO("Obstacle controller [OK]");

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    if (obstacle_controller_active_) {
      computeControls();
      controls_pub_.publish(controls_);

      std_msgs::Float64 potential;
      potential.data = getV();
      potential_pub_.publish(potential);
    }

    rate.sleep();
  }
}

void ObstacleController::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  if (!nh_.getParam("pose_topic", pose_topic_))
    pose_topic_ = "pose";

  if (!nh_.getParam("controls_topic", controls_topic_))
    controls_topic_ = "controls";

  if (!nh_.getParam("obstacles_topic", obstacles_topic_))
    obstacles_topic_ = "obstacles";

  if (!nh_.getParam("potential_topic", potential_topic_))
    potential_topic_ = "potential";

  if (!nh_local_.getParam("world_radius", world_.r))
    world_.r = 3.0;

  if (!nh_local_.getParam("kappa", kappa_))
    kappa_ = 3.0;

  if (!nh_local_.getParam("epsilon", epsilon_))
    epsilon_ = 0.0001;

  if (!nh_local_.getParam("k_w", k_w_))
    k_w_ = 0.1;

  if (!nh_local_.getParam("b_", b_dash_))
    b_dash_ = 2.5;

  if (!nh_local_.getParam("a", a_))
    a_ = 0.5;

  trigger_srv_ = nh_.advertiseService("obstacle_controller_trigger_srv", &ObstacleController::trigger, this);
  params_srv_ = nh_.advertiseService("obstacle_controller_params_srv", &ObstacleController::updateParams, this);
}

double ObstacleController::getBetaWorld() {
  return pow(world_.r, 2.0) - pow(pose_.x - world_.x, 2.0) - pow(pose_.y - world_.y, 2.0);
}

double ObstacleController::getBeta_i(const Obstacle& o) {
  return pow(pose_.x - o.x, 2.0) + pow(pose_.y - o.y, 2.0) - pow(o.r, 2.0);
}

double ObstacleController::getBeta() {
  double beta = getBetaWorld();

  for (auto& o : obstacles_) {
    beta *= getBeta_i(o);
  }

  return beta;
}

vec ObstacleController::getGradBetaWorld() {
  vec gradB_0 = vec(3);

  gradB_0(0) = -2.0 * (pose_.x - world_.x);
  gradB_0(1) = -2.0 * (pose_.y - world_.y);
  gradB_0(2) =  0.0;

  return gradB_0;
}

vec ObstacleController::getGradBeta_i(const Obstacle& o) {
  vec gradB_i = vec(3);

  gradB_i(0) = 2.0 * (pose_.x - o.x);
  gradB_i(1) = 2.0 * (pose_.y - o.y);
  gradB_i(2) = 0.0;

  return gradB_i;
}

vec ObstacleController::getGradBeta() {
  // gradB = gradB0 * B1 * B2 * ... * Bn + B0 * gradB1 * B2 * ... * Bn + ... + B0 * B1 * B2 * ... * gradBn
  vec gradB = vec(3).zeros();        // Total gradient
  vec grad_part = vec(3).zeros();    // Part of gradient

  // First factor: gradB0 * B1 * B2 * ... * Bn
  grad_part = getGradBetaWorld();
  for (int i = 0; i < obstacles_.size(); ++i)
    grad_part *= getBeta_i(obstacles_[i]);

  gradB += grad_part;

  for (int i = 0; i < obstacles_.size(); ++i) {
    grad_part = getGradBeta_i(obstacles_[i]);
    grad_part *= getBetaWorld();

    for (int j = 0; j < obstacles_.size(); ++j) {
      if (i != j) {
        grad_part *= getBeta_i(obstacles_[j]);
      }
    }

    gradB += grad_part;
  }

  return gradB;
}

double ObstacleController::getV() {
  double V = 0.0;

  double squared_norm_r = pow(pose_.x, 2.0) + pow(pose_.y, 2.0);
  double theta_factor = pow(pose_.theta, 2.0) * k_w_ / (k_w_ + squared_norm_r);

  double N = squared_norm_r + theta_factor;
  double D = pow(pow(N, kappa_) + getBeta(), 1.0 / kappa_);

  if (D != 0.0)
    V = N / D;

  return V;
}

vec ObstacleController::getGradV() {
  vec gradV = vec(3).zeros();
  vec gradB = vec(3).zeros();

  double i_kappa = 1.0 / kappa_;

  double squared_norm_r = pow(pose_.x, 2.0) + pow(pose_.y, 2.0);
  double theta_factor = pow(pose_.theta, 2.0) * k_w_ / (k_w_ + squared_norm_r);

  double N = squared_norm_r + theta_factor;             // Numerator of V
  double D = pow(pow(N, kappa_) + getBeta(), i_kappa);  // Denominator of V
  double D_no_kappa = pow(N, kappa_) + getBeta();       // Denominator of V

  // Numerator derivatives
  double dNdx = 2.0 * pose_.x * (1.0 - theta_factor / (k_w_ + squared_norm_r));
  double dNdy = 2.0 * pose_.y * (1.0 - theta_factor / (k_w_ + squared_norm_r));
  double dNdth = 2.0 * pose_.theta * k_w_ / (k_w_ + squared_norm_r);

  // Denominator derivatives
  gradB = getGradBeta();
  double dDdx = i_kappa * pow(D_no_kappa, i_kappa - 1.0) * (kappa_ * pow(N, kappa_ - 1.0) * dNdx + gradB(0));
  double dDdy = i_kappa * pow(D_no_kappa, i_kappa - 1.0) * (kappa_ * pow(N, kappa_ - 1.0) * dNdy + gradB(1));
  double dDdth = i_kappa * pow(D_no_kappa, i_kappa - 1.0) * (kappa_ * pow(N, kappa_ - 1.0) * dNdth + gradB(2));

  if (D != 0.0) {
    gradV(0) = (dNdx * D - N * dDdx) / pow(D, 2.0);     // dV/dx
    gradV(1) = (dNdy * D - N * dDdy) / pow(D, 2.0);     // dV/dy
    gradV(2) = (dNdth * D - N * dDdth) / pow(D, 2.0);   // dV/dtheta
  }

  return gradV;
}

void ObstacleController::computeControls() {
  double b, h, g;   // Variable parameters

  mat B = mat(3, 2).zeros();   // Input matrix
  vec L = vec(3).zeros();      // [sin(fi) -cos(fi) 0]^T
  vec gradV = vec(3).zeros();  // Gradient of navigation function
  vec u_p = vec(2).zeros();    // Control signals time derivative
  vec u = vec(2).zeros();      // Control signals

  // Auxiliary matrices
  arma::mat I = eye<mat>(2, 2);
  arma::mat J = mat(2, 2).zeros();
  J <<  0.0 << 1.0 << endr
    << -1.0 << 0.0 << endr;

  // Update input matrix
  B(0, 0) = cos(pose_.theta);
  B(1, 0) = sin(pose_.theta);
  B(2, 1) = 1.0;

  L(0) =  sin(pose_.theta);
  L(1) = -cos(pose_.theta);

  // Recalculate gradient of navigation function
  gradV = getGradV();

  // Recalculate parameter b
  g = norm(trans(B) * gradV);
  h = pow(g, 2.0) + epsilon_ * sqrt(g);

  if (h != 0.0)
    b = -b_dash_ * dot(L, gradV) / h;
  else
    b = b_dash_;

  // Calculate control signals time derivatives
  u = -(a_ * I + b * J) * trans(B) * gradV;

  controls_.linear.x = u(0);
  controls_.angular.z = u(1);
}

void ObstacleController::poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
  pose_ = *pose_msg;
  pose_.theta = atan2(sin(pose_.theta), cos(pose_.theta));  // Range the orientation in +/- pi
}

void ObstacleController::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg) {
  obstacles_.clear();

  Obstacle o;
  for (int i = 0; i < obstacles_msg->radii.size(); ++i) {
    o.x = obstacles_msg->centre_points[i].x;
    o.y = obstacles_msg->centre_points[i].y;
    o.r = obstacles_msg->radii[i];

    obstacles_.push_back(o);
  }
}

bool ObstacleController::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  obstacle_controller_active_ = req.activate;

  if (req.activate) {
    pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(pose_topic_, 5, &ObstacleController::poseCallback, this);
    obstacles_sub_ = nh_.subscribe<obstacle_detector::Obstacles>(obstacles_topic_, 5, &ObstacleController::obstaclesCallback, this);
    controls_pub_ = nh_.advertise<geometry_msgs::Twist>(controls_topic_, 5);
    potential_pub_ = nh_.advertise<std_msgs::Float64>(potential_topic_, 5);
  }
  else {
    controls_.linear.x = 0.0;
    controls_.angular.z = 0.0;
    controls_pub_.publish(controls_);

    pose_sub_.shutdown();
    obstacles_sub_.shutdown();
    controls_pub_.shutdown();
    potential_pub_.shutdown();
  }

  return true;
}

bool ObstacleController::updateParams(mtracker::Params::Request &req, mtracker::Params::Response &res) {
  // The parameters come as follows:
  // [kappa, epsilon, k_w, b_, a]
  if (req.params.size() >= 5) {
    if (req.params[0] >= 0.0 && req.params[1] >= 0.0 && req.params[2] >= 0.0 && req.params[3] >= 0.0 && req.params[4] >= 0.0) {
      kappa_ = req.params[0];
      epsilon_ = req.params[1];
      k_w_ = req.params[2];
      b_dash_ = req.params[3];
      a_ = req.params[4];

      return true;
    }
    else
      return false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_controller");
  ObstacleController oc;
  return 0;
}
