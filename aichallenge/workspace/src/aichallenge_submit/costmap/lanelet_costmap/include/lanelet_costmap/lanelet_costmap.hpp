// Copyright 2024 Fool Stuck Engineers
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

#ifndef LANELET_COSTMAP__LANELET_COSTMAP_HPP_
#define LANELET_COSTMAP__LANELET_COSTMAP_HPP_

#include <multi_layered_costmap/costmap_base.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace lanelet_costmap
{

using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;

class LaneletCostmap : public multi_layered_costmap::CostmapBase
{
public:
  using SharedPtr = std::shared_ptr<LaneletCostmap>;
  explicit LaneletCostmap(const rclcpp::Node::SharedPtr & node);

  bool is_ready() override;
  bool is_occupied(const geometry_msgs::msg::PointStamped & point) override;

private:
  bool try_transform_point(
    const geometry_msgs::msg::PointStamped & point, geometry_msgs::msg::Point & transformed_point,
    const std::string & target_frame);
  void map_callback(const HADMapBin::SharedPtr msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string map_frame_id_;
  lanelet::LaneletMapPtr map_;
  lanelet::ConstLanelets roads_;
};
}  // namespace lanelet_costmap

#endif  // LANELET_COSTMAP__LANELET_COSTMAP_HPP_
