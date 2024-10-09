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
  
  this->declare_parameter("deceleration", -10.0);
  this->declare_parameter("stop_offset", 3.0);
  this->declare_parameter("max_speed", 30.0);
  this->declare_parameter("traj_width", 1.0);

  deceleration_ = this->get_parameter("deceleration").as_double();
  stop_offset_ = this->get_parameter("stop_offset").as_double();
  max_speed_ = this->get_parameter("max_speed").as_double();
  traj_width_ = this->get_parameter("traj_width").as_double();
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
  const double stop_time= max_speed_/std::abs(deceleration_);
  const double dec_mpss=deceleration_/3.6;
  const double speed_mps=max_speed_/3.6;
  const double stop_dis=0.5*dec_mpss*stop_time*stop_time+speed_mps*stop_time+stop_offset_;
  const int offset_index=stop_dis/traj_width_;
  double v=speed_mps;
  for(int i=0 ; i<offset_index;i++){
    const double t= traj_width_/v;
    v+=dec_mpss*t;
    v=std::max(0.0,v);
    int index = trajectory.points.size() - offset_index + i;
    if (index >= 0 && index < trajectory.points.size()) {
        trajectory.points.at(index).longitudinal_velocity_mps
            = std::min(float(v), trajectory.points.at(index).longitudinal_velocity_mps);
    }
  }
  pub_->publish(trajectory);
}

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
