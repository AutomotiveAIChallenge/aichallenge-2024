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

#include "sensor_converter.hpp"

SensorConverter::SensorConverter(const rclcpp::NodeOptions & node_options)
: Node("sensor_converter", node_options)
{
  using std::placeholders::_1;

  // Parameters
  gnss_pose_delay_ = declare_parameter<int>("gnss_pose_delay");
  gnss_pose_cov_delay_ = declare_parameter<int>("gnss_pose_cov_delay");
  gnss_pose_mean_ = declare_parameter<double>("gnss_pose_mean");
  gnss_pose_stddev_ = declare_parameter<double>("gnss_pose_stddev");
  gnss_pose_cov_mean_ = declare_parameter<double>("gnss_pose_cov_mean");
  gnss_pose_cov_stddev_ = declare_parameter<double>("gnss_pose_cov_stddev");
  imu_mean_ = declare_parameter<double>("imu_mean");
  imu_stddev_ = declare_parameter<double>("imu_stddev");


  // Subscriptions
  sub_gnss_pose_ = create_subscription<Pose>(
    "/awsim/gnss/pose", 1, std::bind(&SensorConverter::on_gnss_pose, this, _1));
  sub_gnss_pose_cov_ = create_subscription<PoseWithCovariance>(
    "/awsim/gnss/pose_with_covariance", 1, std::bind(&SensorConverter::on_gnss_pose_cov, this, _1));
  sub_imu_ = create_subscription<Imu>(
    "/awsim/imu", 1, std::bind(&SensorConverter::on_imu, this, _1));

  // Publishers
  pub_gnss_pose_ = create_publisher<Pose>("/sensing/gnss/pose", 1);
  pub_gnss_pose_cov_ = create_publisher<PoseWithCovariance>("/sensing/gnss/pose_with_covariance", 1);
  pub_imu_ = create_publisher<Imu>("/sensing/imu/imu_raw", 1);

  std::random_device rd;
  generator_ = std::mt19937(rd());
  pose_distribution_ = std::normal_distribution<double>(gnss_pose_mean_, gnss_pose_stddev_);
  pose_cov_distribution_ = std::normal_distribution<double>(gnss_pose_mean_, gnss_pose_stddev_);
  imu_distribution_ = std::normal_distribution<double>(imu_mean_, imu_stddev_);
}

void SensorConverter::on_gnss_pose(const Pose::ConstSharedPtr msg)
{
  rclcpp::sleep_for(std::chrono::milliseconds(gnss_pose_delay_));
  pose_ = std::make_shared<Pose>(*msg);
  // pose_ = msg;
  pose_->position.x += pose_distribution_(generator_);
  pose_->position.y += pose_distribution_(generator_);
  pose_->position.z += pose_distribution_(generator_);
  pose_->orientation.x += pose_distribution_(generator_);
  pose_->orientation.y += pose_distribution_(generator_);
  pose_->orientation.z += pose_distribution_(generator_);
  pose_->orientation.w += pose_distribution_(generator_);

  pub_gnss_pose_->publish(*pose_);
}


void SensorConverter::on_gnss_pose_cov(const PoseWithCovariance::ConstSharedPtr msg)
{
  rclcpp::sleep_for(std::chrono::milliseconds(gnss_pose_cov_delay_));
  pose_cov_ = std::make_shared<PoseWithCovariance>(*msg);
  // pose_cov_ = msg;
  pose_cov_->pose.position.x += pose_cov_distribution_(generator_);
  pose_cov_->pose.position.y += pose_cov_distribution_(generator_);
  pose_cov_->pose.position.z += pose_cov_distribution_(generator_);
  pose_cov_->pose.orientation.x += pose_cov_distribution_(generator_);
  pose_cov_->pose.orientation.y += pose_cov_distribution_(generator_);
  pose_cov_->pose.orientation.z += pose_cov_distribution_(generator_);
  pose_cov_->pose.orientation.w += pose_cov_distribution_(generator_);

  pub_gnss_pose_cov_->publish(*pose_cov_);
}

void SensorConverter::on_imu(const Imu::ConstSharedPtr msg)
{
  imu_ = std::make_shared<Imu>(*msg);
  imu_ -> orientation.x += imu_distribution_(generator_);
  imu_ -> orientation.y += imu_distribution_(generator_);
  imu_ -> orientation.z += imu_distribution_(generator_);
  imu_ -> orientation.w += imu_distribution_(generator_);
  imu_ -> angular_velocity.x += imu_distribution_(generator_);
  imu_ -> angular_velocity.y += imu_distribution_(generator_);
  imu_ -> angular_velocity.z += imu_distribution_(generator_);
  imu_ -> linear_acceleration.x += imu_distribution_(generator_);
  imu_ -> linear_acceleration.y += imu_distribution_(generator_);
  imu_ -> linear_acceleration.z += imu_distribution_(generator_);
  pub_imu_->publish(*imu_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SensorConverter)