// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
#define AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_


#include <random>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;
using sensor_msgs::msg::Imu;

class SensorConverter : public rclcpp::Node
{
public:
  explicit SensorConverter(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<Pose>::SharedPtr sub_gnss_pose_;
  rclcpp::Subscription<PoseWithCovariance>::SharedPtr sub_gnss_pose_cov_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<Pose>::SharedPtr pub_gnss_pose_;
  rclcpp::Publisher<PoseWithCovariance>::SharedPtr pub_gnss_pose_cov_;

  void on_gnss_pose(const Pose::ConstSharedPtr msg);
  void on_gnss_pose_cov(const PoseWithCovariance::ConstSharedPtr msg);
  void on_imu(const Imu::ConstSharedPtr msg);

  Pose::SharedPtr pose_;
  PoseWithCovariance::SharedPtr pose_cov_;
  Imu::SharedPtr imu_;
  int gnss_pose_delay_;
  int gnss_pose_cov_delay_;

  std::mt19937 generator_;
  std::normal_distribution<double> pose_distribution_;
  std::normal_distribution<double> pose_cov_distribution_;
  std;;normal_distribution<double> imu_distribution_;
  double gnss_pose_mean_;
  double gnss_pose_stddev_;
  double gnss_pose_cov_mean_;
  double gnss_pose_cov_stddev_;
  double imu_mean_;
  double imu_stddev_;
};

#endif  // AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
