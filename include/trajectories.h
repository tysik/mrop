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

#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

namespace mrop
{

class Trajectory
{
public:
  Trajectory() : x_0_(0.0), y_0_(0.0), phi_0_(0.0) {}

  Trajectory(double x, double y, double phi) : x_0_(x), y_0_(y), phi_0_(phi) {}

  virtual geometry_msgs::Pose2D calculatePose(double t) {
    geometry_msgs::Pose2D pose;
    pose.x = x_0_;
    pose.y = y_0_;
    pose.theta = phi_0_;

    return pose;
  }

  virtual geometry_msgs::Twist calculateVelocity(double t) {
    geometry_msgs::Twist velocity;
    velocity.linear.x = 0.0;
    velocity.linear.y = 0.0;
    velocity.angular.z = 0.0;

    return velocity;
  }

protected:
  double x_0_;
  double y_0_;
  double phi_0_;
};


class LinearTrajectory : public Trajectory
{
public:
  LinearTrajectory() : v_(0.0) {}

  LinearTrajectory(double phi, double v) : Trajectory(0.0, 0.0, phi), v_(v) {}

  LinearTrajectory(double x, double y, double phi, double v) : Trajectory(x, y, phi), v_(v) {}

  virtual geometry_msgs::Pose2D calculatePose(double t) {
    geometry_msgs::Pose2D pose;
    pose.x = x_0_ + v_ * cos(phi_0_) * t;
    pose.y = y_0_ + v_ * sin(phi_0_) * t;
    pose.theta = phi_0_;

    return pose;
  }

  virtual geometry_msgs::Twist calculateVelocity(double t) {
    geometry_msgs::Twist velocity;
    velocity.linear.x = v_ * cos(phi_0_);
    velocity.linear.y = v_ * sin(phi_0_);
    velocity.angular.z = 0.0;

    return velocity;
  }

protected:
  double v_;    // Speed [m/s]
};


class HarmonicTrajectory : public Trajectory
{
public:
  HarmonicTrajectory() : w_(0.0), r_x_(0.0), r_y_(0.0), n_x_(0.0), n_y_(0.0) {}

  HarmonicTrajectory(double T, double r_x, double r_y, int n_x, int n_y) :
    r_x_(r_x), r_y_(r_y), n_x_(n_x), n_y_(n_y)
    { (T > 0.0) ? w_ = 2.0 * M_PI / T : w_ = 0.0; }

  HarmonicTrajectory(double x, double y, double T, double r_x, double r_y, int n_x, int n_y) :
    Trajectory(x, y, 0.0), r_x_(r_x), r_y_(r_y), n_x_(n_x), n_y_(n_y)
    { (T > 0.0) ? w_ = 2.0 * M_PI / T : w_ = 0.0; }

  virtual geometry_msgs::Pose2D calculatePose(double t) {
    geometry_msgs::Pose2D pose;
    pose.x = x_0_ + r_x_ * cos(n_x_ * w_ * t);
    pose.y = y_0_ + r_y_ * sin(n_y_ * w_ * t);
    pose.theta = atan2(r_y_ * n_y_ * w_ * cos(n_y_ * w_ * t), -r_x_ * n_x_ * w_ * sin(n_x_ * w_ * t));

    return pose;
  }

  virtual geometry_msgs::Twist calculateVelocity(double t) {
    geometry_msgs::Twist velocity;
    velocity.linear.x = -r_x_ * n_x_ * w_ * sin(n_x_ * w_ * t);
    velocity.linear.y =  r_y_ * n_y_ * w_ * cos(n_y_ * w_ * t);

    if (pow(velocity.linear.x, 2.0) + pow(velocity.linear.y, 2.0) > 0.0)
      velocity.angular.z = (-r_y_ * pow(n_y_, 2.0) * pow(w_, 2.0) * sin(n_y_ * w_ * t) * velocity.linear.x -
                  -r_x_ * pow(n_x_, 2.0) * pow(w_, 2.0) * cos(n_x_ * w_ * t) * velocity.linear.y) /
                 (pow(velocity.linear.x, 2.0) + pow(velocity.linear.y, 2.0));

    return velocity;
  }

protected:
  double w_;           // Frequency [rad/s]
  double r_x_, r_y_;   // Radii [m]
  int n_x_, n_y_;      // Frequency multipliers [-]
};


class LemniscateTrajectory : public Trajectory
{
public:
  LemniscateTrajectory() : w_(0.0), r_x_(0.0), r_y_(0.0), a_(0.0), b_(0.0) {}

  LemniscateTrajectory(double T, double r_x, double r_y, double a, double b) :
    r_x_(r_x), r_y_(r_y), a_(a), b_(b)
    { (T > 0.0) ? w_ = 2 * M_PI / T : w_ = 0.0; }

  LemniscateTrajectory(double x, double y, double T, double r_x, double r_y, double a, double b) :
    Trajectory(x, y, 0.0), r_x_(r_x), r_y_(r_y), a_(a), b_(b)
    { (T > 0.0) ? w_ = 2 * M_PI / T : w_ = 0.0; }

  virtual geometry_msgs::Pose2D calculatePose(double t) {
    geometry_msgs::Pose2D pose;

    double B_aux = pow(b_, 4.0) - pow(a_, 2.0) * pow(sin(2.0 * w_ * t), 2.0);
    double B;
    if (B_aux >= 0.0)
      B = sqrt(B_aux);
    else
      B = sqrt(-B_aux);

    double C = pow(a_, 2.0) * cos(2.0 * w_ * t) + B;
    double dC_dt;
    if (C != 0.0 && B != 0.0)
      dC_dt = -(pow(a_, 2.0) * w_ / C) * (sin(2.0 * w_ * t) + sin(4.0 * w_ * t) / (2.0 * B));
    else
      dC_dt = 1.0;

    double vx = -r_x_ * w_ * sin(w_ * t) * C + r_x_ * cos(w_ * t) * dC_dt;
    double vy =  r_y_ * w_ * cos(w_ * t) * C + r_x_ * sin(w_ * t) * dC_dt;

    pose.x = r_x_ * cos(w_ * t) * C;
    pose.y = r_y_ * sin(w_ * t) * C;
    pose.theta = atan2(vy, vx);

    return pose;
  }

  virtual geometry_msgs::Twist calculateVelocity(double t) {
    geometry_msgs::Twist velocity;

    double B_aux = pow(b_, 4.0) - pow(a_, 2.0) * pow(sin(2.0 * w_ * t), 2.0);
    double B;
    if (B_aux >= 0.0)
      B = sqrt(B_aux);
    else
      B = sqrt(-B_aux);

    double C = pow(a_, 2.0) * cos(2.0 * w_ * t) + B;
    double dC_dt;
    if (C != 0.0 && B != 0.0)
      dC_dt = -(pow(a_, 2.0) * w_ / C) * (sin(2.0 * w_ * t) + sin(4.0 * w_ * t) / (2.0 * B));
    else
      dC_dt = 1.0;

    velocity.linear.x = -r_x_ * w_ * sin(w_ * t) * C + r_x_ * cos(w_ * t) * dC_dt;
    velocity.linear.y =  r_y_ * w_ * cos(w_ * t) * C + r_x_ * sin(w_ * t) * dC_dt;
    velocity.angular.z = 0.0; // TODO calculate this

    return velocity;
  }

protected:
  double w_;      // Frequency [rad/s]
  double r_x_;    // Radii [m]
  double r_y_;
  double a_;      // Separation of focal [m]
  double b_;      // Distance (product) [m]
};

} // namespace mrop
