// Copyright 2024 Booars
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

#ifndef BOOARS_COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define BOOARS_COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include <booars_costmap_utils/booars_costmap_utils.hpp>
#include <booars_utils/nav/occupancy_grid_parameters.hpp>
#include <booars_utils/ros/function_timer.hpp>
#include <multi_layered_costmap/multi_layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace booars_costmap_generator
{

using FunctionTimer = booars_utils::ros::FunctionTimer;
using MultiLayeredCostmap = multi_layered_costmap::MultiLayeredCostmap;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using OccupancyGridParameters = booars_utils::nav::OccupancyGridParameters;
using PointStamped = geometry_msgs::msg::PointStamped;
using Point2d = tier4_autoware_utils::Point2d;
using Vector3 = geometry_msgs::msg::Vector3;

class CostmapGenerator : public rclcpp::Node
{
public:
  explicit CostmapGenerator(const rclcpp::NodeOptions & options);

private:
  void update();
  bool try_get_transform(geometry_msgs::msg::TransformStamped & transform);

  FunctionTimer::SharedPtr update_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<OccupancyGrid>::SharedPtr costmap_pub_;
  MultiLayeredCostmap::SharedPtr multi_layered_costmap_;

  OccupancyGrid::SharedPtr costmap_;
  OccupancyGridParameters::SharedPtr costmap_parameters_;

  std::string map_frame_;
  std::string target_frame_;

  std::vector<Point2d> index_to_point_table_;
};
};  // namespace booars_costmap_generator

#endif  // BOOARS_COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
