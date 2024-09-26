// Copyright 2024 TIER IV, Inc.
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using sensor_msgs::msg::Imu;
using autoware_auto_vehicle_msgs::msg::SteeringReport;

class SensorConverter : public rclcpp::Node
{
public:
  explicit SensorConverter(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_gnss_pose_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_cov_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_report_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_gnss_pose_;
  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_gnss_pose_cov_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_report_;


  void on_gnss_pose(const PoseStamped::ConstSharedPtr msg);
  void on_gnss_pose_cov(const PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_imu(const Imu::ConstSharedPtr msg);
  void on_steering_report(const SteeringReport::ConstSharedPtr msg);

  PoseStamped::SharedPtr pose_;
  PoseWithCovarianceStamped::SharedPtr pose_cov_;
  Imu::SharedPtr imu_;
  SteeringReport::SharedPtr steering_report_;
  int gnss_pose_delay_;
  int gnss_pose_cov_delay_;

  std::mt19937 generator_;
  std::normal_distribution<double> pose_distribution_;
  std::normal_distribution<double> pose_cov_distribution_;
  std::normal_distribution<double> imu_acc_distribution_;
  std::normal_distribution<double> imu_ang_distribution_;
  std::normal_distribution<double> imu_ori_distribution_;
  std::normal_distribution<double> steering_angle_distribution_;
  double gnss_pose_mean_;
  double gnss_pose_stddev_;
  double gnss_pose_cov_mean_;
  double gnss_pose_cov_stddev_;
  double imu_acc_mean_;
  double imu_acc_stddev_;
  double imu_ang_mean_;
  double imu_ang_stddev_;
  double imu_ori_mean_;
  double imu_ori_stddev_;
  double steering_angle_mean_;
  double steering_angle_stddev_;
};

#endif  // AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
