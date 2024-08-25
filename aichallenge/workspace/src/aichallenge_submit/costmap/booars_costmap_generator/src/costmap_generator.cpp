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

#include "booars_costmap_generator/costmap_generator.hpp"

#include <booars_utils/nav/occupancy_grid_utils.hpp>

namespace booars_costmap_generator
{
CostmapGenerator::CostmapGenerator(const rclcpp::NodeOptions & options)
: Node("costmap_generator", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  // Declare parameters
  {
    map_frame_ = this->declare_parameter("map_frame_id", "map");
    target_frame_ = this->declare_parameter("costmap_center_frame_id", "base_link");
  }

  // Create publisher
  {
    costmap_pub_ = this->create_publisher<OccupancyGrid>("~/output/costmap", 1);
  }

  // Crate update() timer
  {
    double update_rate = this->declare_parameter("update_rate", 10.0);
    update_timer_ = FunctionTimer::create_function_timer(
      this, update_rate, std::bind(&CostmapGenerator::update, this));
  }

  // Declare costmap parameters
  {
    double costmap_width = this->declare_parameter("costmap_width", 10.0);
    double costmap_resolution = this->declare_parameter("costmap_resolution", 0.1);
    costmap_parameters_ =
      OccupancyGridParameters::create_parameters(costmap_width, costmap_resolution);
    index_to_point_table_ =
      booars_utils::nav::occupancy_grid_utils::get_index_to_point_table(costmap_parameters_);
  }

  // Create costmap
  {
    costmap_ = booars_utils::nav::occupancy_grid_utils::create_occupancy_grid(costmap_parameters_);
    costmap_->header.frame_id = map_frame_;
  }

  // Crate multi layered costmap
  {
    multi_layered_costmap_ =
      booars_costmap_utils::create_multi_layered_costmap(*this, "multi_layered_costmap");
  }
}

void CostmapGenerator::update()
{
  if (!multi_layered_costmap_->is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "MultiLayeredCostmap is not ready");
    return;
  }

  rclcpp::Time costmap_time;
  Vector3 costmap_center_position;
  {
    geometry_msgs::msg::TransformStamped transform;
    if (!try_get_transform(transform)) return;
    costmap_time = transform.header.stamp;
    costmap_center_position = transform.transform.translation;
  }

  // Fill the costmap data
  {
    for (int i = 0; i < costmap_parameters_->grid_num(); i++) {
      Point2d local_point = index_to_point_table_[i];
      PointStamped global_point;
      global_point.header.frame_id = map_frame_;
      global_point.header.stamp = costmap_time;
      global_point.point.x = local_point[0] + costmap_center_position.x;
      global_point.point.y = local_point[1] + costmap_center_position.y;
      global_point.point.z = 0.0;
      costmap_->data[i] = multi_layered_costmap_->is_occupied(global_point) ? 100 : 0;
    }
  }

  // Update the costmap origin
  {
    booars_utils::nav::occupancy_grid_utils::update_origin(
      costmap_, costmap_parameters_, costmap_center_position);
  }

  // Publish the costmap
  {
    costmap_->header.stamp = costmap_time;
    costmap_pub_->publish(*costmap_);
  }
}

bool CostmapGenerator::try_get_transform(geometry_msgs::msg::TransformStamped & transform)
{
  try {
    transform = tf_buffer_.lookupTransform(map_frame_, target_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Failed to get transform: %s", ex.what());
    return false;
  }
  return true;
}
}  // namespace booars_costmap_generator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(booars_costmap_generator::CostmapGenerator)
