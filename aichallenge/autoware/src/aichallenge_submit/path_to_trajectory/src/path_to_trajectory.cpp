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

PathToTrajectory::PathToTrajectory() : Node("path_to_trajectory_node") {
  using std::placeholders::_1;

  pub_ = this->create_publisher<Trajectory>("output", 1);
  sub_ = this->create_subscription<PathWithLaneId>(
      "input", 1, std::bind(&PathToTrajectory::callback, this, _1));
}

void PathToTrajectory::callback(const PathWithLaneId::SharedPtr msg) {
  Trajectory trajectory;
  trajectory.header = msg->header;
  for (auto& path_point_with_lane_id : msg->points) {
    TrajectoryPoint trajectory_point;
    trajectory_point.pose = path_point_with_lane_id.point.pose;
    trajectory_point.longitudinal_velocity_mps = path_point_with_lane_id.point.longitudinal_velocity_mps;
    trajectory.points.emplace_back(std::move(trajectory_point));
  }
  pub_->publish(trajectory);
}

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
