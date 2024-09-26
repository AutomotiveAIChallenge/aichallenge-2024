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
  imu_acc_mean_ = declare_parameter<double>("imu_acc_mean");
  imu_acc_stddev_ = declare_parameter<double>("imu_acc_stddev");
  imu_ang_mean_ = declare_parameter<double>("imu_ang_mean");
  imu_ang_stddev_ = declare_parameter<double>("imu_ang_stddev");
  imu_ori_mean_ = declare_parameter<double>("imu_ori_mean");
  imu_ori_stddev_ = declare_parameter<double>("imu_ori_stddev");
  steering_angle_mean_ = declare_parameter<double>("steering_angle_mean");
  steering_angle_stddev_ = declare_parameter<double>("steering_angle_stddev");


  // Subscriptions
  sub_gnss_pose_ = create_subscription<PoseStamped>(
    "/awsim/gnss/pose", 1, std::bind(&SensorConverter::on_gnss_pose, this, _1));
  sub_gnss_pose_cov_ = create_subscription<PoseWithCovarianceStamped>(
    "/awsim/gnss/pose_with_covariance", 1, std::bind(&SensorConverter::on_gnss_pose_cov, this, _1));
  sub_imu_ = create_subscription<Imu>(
    "/awsim/imu", 1, std::bind(&SensorConverter::on_imu, this, _1));
  sub_steering_report_ = create_subscription<SteeringReport>(
    "/awsim/steering_status", 1, std::bind(&SensorConverter::on_steering_report, this, _1));

  // Publishers
  pub_gnss_pose_ = create_publisher<PoseStamped>("/sensing/gnss/pose", 1);
  pub_gnss_pose_cov_ = create_publisher<PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance", 1);
  pub_imu_ = create_publisher<Imu>("/sensing/imu/imu_raw", 1);
  pub_steering_report_ = create_publisher<SteeringReport>("/vehicle/status/steering_status", 1);

  std::random_device rd;
  generator_ = std::mt19937(rd());
  pose_distribution_ = std::normal_distribution<double>(gnss_pose_mean_, gnss_pose_stddev_);
  pose_cov_distribution_ = std::normal_distribution<double>(gnss_pose_mean_, gnss_pose_stddev_);
  imu_acc_distribution_ = std::normal_distribution<double>(imu_acc_mean_, imu_acc_stddev_);
  imu_ang_distribution_ = std::normal_distribution<double>(imu_ang_mean_, imu_ang_stddev_);
  imu_ori_distribution_ = std::normal_distribution<double>(imu_ori_mean_, imu_ori_stddev_);
  steering_angle_distribution_ = std::normal_distribution<double>(steering_angle_mean_, steering_angle_stddev_);
}

void SensorConverter::on_gnss_pose(const PoseStamped::ConstSharedPtr msg)
{
  auto process_and_publish_gnss = [this, msg]() {
    rclcpp::sleep_for(std::chrono::milliseconds(gnss_pose_delay_));
    
    auto pose = std::make_shared<PoseStamped>(*msg);
    pose->header.stamp = now();
    pose->pose.position.x += pose_distribution_(generator_);
    pose->pose.position.y += pose_distribution_(generator_);
    pose->pose.position.z += pose_distribution_(generator_);
    pose->pose.orientation.x += pose_distribution_(generator_);
    pose->pose.orientation.y += pose_distribution_(generator_);
    pose->pose.orientation.z += pose_distribution_(generator_);
    pose->pose.orientation.w += pose_distribution_(generator_);

    pub_gnss_pose_->publish(*pose);
  };

  std::thread processing_thread(process_and_publish_gnss);
  processing_thread.detach();
}


void SensorConverter::on_gnss_pose_cov(const PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    auto process_and_publish_gnss_cov = [this, msg]() {
    rclcpp::sleep_for(std::chrono::milliseconds(gnss_pose_cov_delay_));
    
    auto pose_cov = std::make_shared<PoseWithCovarianceStamped>(*msg);
    pose_cov->header.stamp = now();
    pose_cov->pose.pose.position.x += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.position.y += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.position.z += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.orientation.x += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.orientation.y += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.orientation.z += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.orientation.w += pose_cov_distribution_(generator_);

    pub_gnss_pose_cov_->publish(*pose_cov);
  };

  std::thread processing_thread(process_and_publish_gnss_cov);
  processing_thread.detach();
}

void SensorConverter::on_imu(const Imu::ConstSharedPtr msg)
{
  imu_ = std::make_shared<Imu>(*msg);
  imu_ -> orientation.x += imu_ori_distribution_(generator_);
  imu_ -> orientation.y += imu_ori_distribution_(generator_);
  imu_ -> orientation.z += imu_ori_distribution_(generator_);
  imu_ -> orientation.w += imu_ori_distribution_(generator_);
  imu_ -> angular_velocity.x += imu_ang_distribution_(generator_);
  imu_ -> angular_velocity.y += imu_ang_distribution_(generator_);
  imu_ -> angular_velocity.z += imu_ang_distribution_(generator_);
  imu_ -> linear_acceleration.x += imu_acc_distribution_(generator_);
  imu_ -> linear_acceleration.y += imu_acc_distribution_(generator_);
  imu_ -> linear_acceleration.z += imu_acc_distribution_(generator_);
  pub_imu_->publish(*imu_);
}

void SensorConverter::on_steering_report(const SteeringReport::ConstSharedPtr msg)
{
  steering_report_ = std::make_shared<SteeringReport>(*msg);
  steering_report_->steering_tire_angle += steering_angle_distribution_(generator_);
  pub_steering_report_->publish(*steering_report_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SensorConverter)
