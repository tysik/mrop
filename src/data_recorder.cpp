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

#include "../include/data_recorder.h"

using namespace mtracker;

DataRecorder::DataRecorder() : nh_(""), nh_local_("~") {
  initialize();

  ROS_INFO("MTracker data recorder [OK]");

  ros::Rate rate(loop_rate_);
  while (nh_.ok()) {
    ros::spinOnce();

    if (recording_) {
      addLatestData();
    }

    rate.sleep();
  }
}

DataRecorder::~DataRecorder() {
  if (recording_)
    stop();
}

void DataRecorder::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  if (!nh_.getParam("pose_topic", pose_topic_))
    pose_topic_ = "pose";

  if (!nh_.getParam("reference_pose_topic", reference_pose_topic_))
    reference_pose_topic_ = "reference_pose";

  if (!nh_.getParam("controls_topic", controls_topic_))
    controls_topic_ = "controls";

  if (!nh_.getParam("scaled_controls_topic", scaled_controls_topic_))
    scaled_controls_topic_ = "scaled_controls";

  if (!nh_.getParam("obstacles_topic", obstacles_topic_))
    obstacles_topic_ = "obstacles";

  if (!nh_.getParam("potential_topic", potential_topic_))
    potential_topic_ = "potential";

  if (!nh_local_.getParam("use_txt", use_txt_))
    use_txt_ = true;

  if (!nh_local_.getParam("use_yaml", use_yaml_))
    use_yaml_ = false;

  if (!nh_local_.getParam("record_pose", record_pose_))
    record_pose_ = true;

  if (!nh_local_.getParam("record_reference_pose", record_reference_pose_))
    record_reference_pose_ = true;

  if (!nh_local_.getParam("record_controls", record_controls_))
    record_controls_ = true;

  if (!nh_local_.getParam("record_scaled_controls", record_scaled_controls_))
    record_scaled_controls_ = true;

  if (!nh_local_.getParam("record_obstacles", record_obstacles_))
    record_obstacles_ = true;

  if (!nh_local_.getParam("record_potential", record_potential_))
    record_potential_ = true;

  trigger_srv_ = nh_.advertiseService("data_recorder_trigger_srv", &DataRecorder::trigger, this);
  params_srv_ = nh_.advertiseService("data_recorder_params_srv", &DataRecorder::updateParams, this);

  std::string username = getenv("USER");
  std::string folder = "/home/" + username + "/MTrackerRecords/";
  boost::filesystem::create_directories(folder);

  potential_ = 0.0;
}

void DataRecorder::start() {
  if (recording_)
    return;

  start_mark_ = ros::Time::now();
  recording_ = true;
}

void DataRecorder::stop() {
  if (!recording_)
    return;

  recording_ = false;

  if (use_yaml_)
    emitYamlFile();
  if (use_txt_)
    emitTxtFile();

  t_.clear();
  pose_list_.clear();
  ref_pose_list_.clear();
  controls_list_.clear();
  scaled_controls_list_.clear();
  potential_list_.clear();
}

void DataRecorder::addLatestData() {
  double t = (ros::Time::now() - start_mark_).toSec();
  t_.push_back(t);
  pose_list_.push_back(pose_);
  ref_pose_list_.push_back(ref_pose_);
  controls_list_.push_back(controls_);
  scaled_controls_list_.push_back(scaled_controls_);
  potential_list_.push_back(potential_);
}

void DataRecorder::emitYamlFile() {
/*  static int file_number = 0;
  ++file_number;
  YAML::Emitter emitter;

  time_t actual_time = time(nullptr);
  char header_info[100];
  strftime(header_info, 100, "This data was recorded on %F at %T.", gmtime(&actual_time));

  emitter << YAML::Comment(header_info);
  emitter << YAML::BeginSeq;
  emitter << YAML::Anchor("Pose");
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::Key << "y" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::Key << "theta" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::EndMap;
  emitter << YAML::Anchor("Velocity");
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "v" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::Key << "w" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::EndMap;
  emitter << YAML::Anchor("Time");
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "t" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::EndMap;
  emitter << YAML::EndSeq;

  std::string username = getenv("USER");
  std::string filename = "/home/" + username + "/MTrackerRecords/MTrackerRecord_" + std::to_string(file_number) + ".yaml";
  std::ofstream file(filename);

  file << emitter.c_str();
  file.close();*/
}

void DataRecorder::emitTxtFile() {
  time_t now = time(NULL);
  char the_date[30];

  if (now != -1)
    strftime(the_date, 30, "%Y_%m_%d_%H_%M_%S", gmtime(&now));

  std::string username = getenv("USER");
  std::string filename = "/home/" + username + "/MTrackerRecords/MTrackerRecord_" + std::string(the_date) + ".txt";
  std::ofstream file(filename);

  // Create header line
  file << "n \t t";
  if (record_pose_)
    file << " \t x \t y \t theta";
  if (record_reference_pose_)
    file << " \t x_r \t y_r \t theta_r";
  if (record_controls_)
    file << " \t v \t w";
  if (record_scaled_controls_)
    file << " \t v_s \t w_s";
  if (record_potential_)
    file << " \t P";
  file << "\n";

  for (int i = 0; i < t_.size(); ++i) {
    file << i << "\t" << t_[i];
    if (record_pose_)
      file << "\t" << pose_list_[i].x << "\t" << pose_list_[i].y << "\t" << pose_list_[i].theta;
    if (record_reference_pose_)
      file << "\t" << ref_pose_list_[i].x << "\t" << ref_pose_list_[i].y << "\t" << ref_pose_list_[i].theta;
    if (record_controls_)
      file << "\t" << controls_list_[i].linear.x << "\t" << controls_list_[i].angular.z;
    if (record_scaled_controls_)
      file << "\t" << scaled_controls_list_[i].linear.x << "\t" << scaled_controls_list_[i].angular.z;
    if (record_potential_)
      file << "\t" << potential_list_[i];
    file << "\n";
  }

  file.close();

  if (record_obstacles_) {
    std::string obstacles_filename = "/home/" + username + "/MTrackerRecords/ObstaclesRecord_" + std::string(the_date) + ".txt";
    std::ofstream obstacles_file(obstacles_filename);

    for (int i = 0; i < obstacles_list_.size(); ++i) {
      for (int j = 0; j < obstacles_list_[i].size(); ++j) {
        obstacles_file << obstacles_list_[i][j].x << "\t" << obstacles_list_[i][j].y << "\t" << obstacles_list_[i][j].r << "\t";
      }
      // Fill row to 40 (padding)
      for (int j = obstacles_list_[i].size(); j < 40; ++j) {
        obstacles_file << "0.0 \t 0.0 \t 0.0 \t";
      }
      obstacles_file << "\n";
    }

    obstacles_file.close();
  }
}

void DataRecorder::poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
  pose_ = *pose_msg;
}

void DataRecorder::refPoseCallback(const geometry_msgs::Pose2D::ConstPtr& ref_pose_msg) {
  ref_pose_ = *ref_pose_msg;
}

void DataRecorder::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

void DataRecorder::scaledControlsCallback(const geometry_msgs::Twist::ConstPtr& scaled_controls_msg) {
  scaled_controls_ = *scaled_controls_msg;
}

void DataRecorder::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr &obstacles_msg) {
  if (record_obstacles_) {
    std::vector<Obstacle> o_list;

    Obstacle o;
    for (int i = 0; i < obstacles_msg->radii.size(); ++i) {
      o.x = obstacles_msg->centre_points[i].x;
      o.y = obstacles_msg->centre_points[i].y;
      o.r = obstacles_msg->radii[i];

      o_list.push_back(o);
    }

    obstacles_list_.push_back(o_list);
  }
}

void DataRecorder::potentialCallback(const std_msgs::Float64::ConstPtr& potential_msg) {
  potential_ = potential_msg->data;
}

bool DataRecorder::trigger(mtracker::Trigger::Request& req, mtracker::Trigger::Response& res) {
  if (req.activate) {
    pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(pose_topic_, 5, &DataRecorder::poseCallback, this);
    ref_pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(reference_pose_topic_, 5, &DataRecorder::refPoseCallback, this);
    controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(controls_topic_, 5, &DataRecorder::controlsCallback, this);
    scaled_controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(scaled_controls_topic_, 5, &DataRecorder::scaledControlsCallback, this);
    obstacles_sub_ = nh_.subscribe<obstacle_detector::Obstacles>(obstacles_topic_, 5, &DataRecorder::obstaclesCallback, this);
    potential_sub_ = nh_.subscribe<std_msgs::Float64>(potential_topic_, 5, &DataRecorder::potentialCallback, this);
  }
  else {
    stop();

    pose_sub_.shutdown();
    ref_pose_sub_.shutdown();
    controls_sub_.shutdown();
    scaled_controls_sub_.shutdown();
    obstacles_sub_.shutdown();
    potential_sub_.shutdown();
  }

  return true;
}

bool DataRecorder::updateParams(mtracker::Params::Request& req, mtracker::Params::Response& res) {
  // The parameters come as follows:
  // [recording, rec_pose, rec_ref_pose, rec_ctrls, rec_scal_ctrls, rec_obstac, rec_poten]
  if (req.params.size() >= 7) {
    record_pose_ = static_cast<bool>(req.params[1]);
    record_reference_pose_ = static_cast<bool>(req.params[2]);
    record_controls_ = static_cast<bool>(req.params[3]);
    record_scaled_controls_ = static_cast<bool>(req.params[4]);
    record_obstacles_ = static_cast<bool>(req.params[5]);
    record_potential_ = static_cast<bool>(req.params[6]);

    if (static_cast<bool>(req.params[0]))
      start();
    else
      stop();

    return true;
  }
  else
    return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_recorder");
  mtracker::DataRecorder dr;
  return 0;
}
