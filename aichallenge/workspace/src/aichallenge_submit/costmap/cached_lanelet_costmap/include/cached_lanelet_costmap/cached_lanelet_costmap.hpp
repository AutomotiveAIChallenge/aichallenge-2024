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

#ifndef CACHED_LANELET_COSTMAP__CACHED_LANELET_COSTMAP_HPP_
#define CACHED_LANELET_COSTMAP__CACHED_LANELET_COSTMAP_HPP_

#include <booars_utils/nav/occupancy_grid_parameters.hpp>
#include <booars_utils/nav/occupancy_grid_utils.hpp>
#include <multi_layered_costmap/costmap_base.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace cached_lanelet_costmap
{

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using OccupancyGridParameters = booars_utils::nav::OccupancyGridParameters;
using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;

class CachedLaneletCostmap : public multi_layered_costmap::CostmapBase
{
public:
  using SharedPtr = std::shared_ptr<CachedLaneletCostmap>;
  explicit CachedLaneletCostmap(rclcpp::Node & node, const std::string & layer_namespace);

  static CostmapBase::SharedPtr create_costmap(
    rclcpp::Node & node, const std::string & layer_namespace)
  {
    return std::make_shared<CachedLaneletCostmap>(node, layer_namespace);
  }

  bool is_ready() override;
  bool is_occupied(const geometry_msgs::msg::PointStamped & point) override;

private:
  bool try_transform_point(
    const geometry_msgs::msg::PointStamped & point, geometry_msgs::msg::Point & transformed_point,
    const std::string & target_frame);
  void create_cached_costmap(
    const std::string & map_frame_id, const lanelet::ConstLanelets & roads_lanelets);
  void map_callback(const HADMapBin::SharedPtr msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<HADMapBin>::SharedPtr map_sub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr costmap_pub_;

  OccupancyGrid::SharedPtr cached_costmap_;
  OccupancyGridParameters::SharedPtr cached_costmap_parameters_;

  double cached_costmap_origin_x_;
  double cached_costmap_origin_y_;
  bool is_map_ready_ = false;
};
}  // namespace cached_lanelet_costmap

#endif  // CACHED_LANELET_COSTMAP__CACHED_LANELET_COSTMAP_HPP_
