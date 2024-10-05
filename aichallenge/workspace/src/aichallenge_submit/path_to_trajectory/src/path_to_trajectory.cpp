// Copyright 2023 Tier IV, Inc. All rights reserved.
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

#include "path_to_trajectory/path_to_trajectory.hpp"
#include <fstream>
#include <sstream>
#include <string>

PathToTrajectory::PathToTrajectory() : Node("path_to_trajectory_node")
{
  RCLCPP_INFO(this->get_logger(), "================ Path To Trajectory ==================");

  this->declare_parameter("base_path", std::string("base_path"));
  base_path_ = this->get_parameter("base_path").as_string();

  RCLCPP_INFO(this->get_logger(), "base_path: %s", base_path_.c_str());

  trajectory_pub_ = this->create_publisher<Trajectory>("/planning/scenario_planning/trajectory", 1);

  load_csv(base_path_);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PathToTrajectory::callback, this)
  );

  set_trajectory_srv_ = this->create_service<csv_path_changer_msgs::srv::SetTrajectory>(
    "/set_trajectory",
    std::bind(&PathToTrajectory::handle_trajectory, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void PathToTrajectory::callback() {
  trajectory_pub_->publish(trajectory_);
}

void PathToTrajectory::handle_trajectory(
  const std::shared_ptr<csv_path_changer_msgs::srv::SetTrajectory::Request> request,
  std::shared_ptr<csv_path_changer_msgs::srv::SetTrajectory::Response> response)
{
  write_csv(request->csv_path, request->points);
  load_csv(request->csv_path);
  response->success = true;
}

void PathToTrajectory::write_csv(const std::string &csv_path, const std::vector<TrajectoryPoint> &points)
{
  std::ofstream file(csv_path);
  
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing: %s", csv_path.c_str());
    return;
  }
  
  // ヘッダーを書き込む（必要に応じて変更）
  file << "x,y,z,x_quat,y_quat,z_quat,w_quat,speed\n";
  
  // 各ポイントを書き込む
  for (const auto &point : points) {
    const auto &pose = point.pose;
    file << pose.position.x << ","
         << pose.position.y << ","
         << pose.position.z << ","
         << pose.orientation.x << ","
         << pose.orientation.y << ","
         << pose.orientation.z << ","
         << pose.orientation.w << ","
         << point.longitudinal_velocity_mps << "\n";
  }
  
  file.close();
  RCLCPP_INFO(this->get_logger(), "Successfully wrote points to CSV: %s", csv_path.c_str());
}

void PathToTrajectory::load_csv(std::string csv_path)
{
  RCLCPP_INFO(this->get_logger(), "--------------- Load CSV %s ---------------", csv_path.c_str());

  std::ifstream file(csv_path);
  std::string line;
  if (!file.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Failed to open CSV file");
  } else {
    RCLCPP_INFO(this->get_logger(), "Reading CSV file");
    std::getline(file, line);

    trajectory_ = Trajectory();
    trajectory_.header.stamp = this->now();
    trajectory_.header.frame_id = "map";

    while (std::getline(file, line))
    {
      std::stringstream ss(line);
      std::string x, y, z, x_quat, y_quat, z_quat, w_quat, speed;
      std::getline(ss, x, ',');
      std::getline(ss, y, ',');
      std::getline(ss, z, ',');
      std::getline(ss, x_quat, ',');
      std::getline(ss, y_quat, ',');
      std::getline(ss, z_quat, ',');
      std::getline(ss, w_quat, ',');
      std::getline(ss, speed, ',');

      geometry_msgs::msg::Pose pose;
      pose.position.x = std::stof(x);
      pose.position.y = std::stof(y);
      pose.position.z = 43.1;
      pose.orientation.x = std::stof(x_quat);
      pose.orientation.y = std::stof(y_quat);
      pose.orientation.z = std::stof(z_quat);
      pose.orientation.w = std::stof(w_quat);

      TrajectoryPoint point;
      point.pose = pose;
      point.longitudinal_velocity_mps = std::stof(speed);
      trajectory_.points.push_back(point);
    }
    file.close();
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu points", trajectory_.points.size());
}

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
