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

#include "lanelet_costmap/lanelet_costmap.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace lanelet_costmap
{
LaneletCostmap::LaneletCostmap(rclcpp::Node & node, const std::string & layer_namespace)
: tf_buffer_(node.get_clock()), tf_listener_(tf_buffer_)
{
  std::string map_topic = node.declare_parameter(layer_namespace + ".map_topic", "~/input/map");
  map_sub_ = node.create_subscription<HADMapBin>(
    map_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&LaneletCostmap::map_callback, this, std::placeholders::_1));
}

bool LaneletCostmap::is_ready()
{
  return map_ != nullptr;
}

bool LaneletCostmap::is_occupied(const geometry_msgs::msg::PointStamped & point)
{
  if (!is_ready()) return true;

  geometry_msgs::msg::Point transformed_point;
  if (!try_transform_point(point, transformed_point, map_frame_id_)) {
    return true;
  }

  tier4_autoware_utils::Point2d point2d(transformed_point.x, transformed_point.y);
  for (const auto & road : roads_) {
    if (!lanelet::geometry::within(point2d, road.polygon2d().basicPolygon())) continue;
    return false;
  }
  return true;
}

bool LaneletCostmap::try_transform_point(
  const geometry_msgs::msg::PointStamped & point, geometry_msgs::msg::Point & transformed_point,
  const std::string & target_frame)
{
  geometry_msgs::msg::PointStamped transformed_point_stamped;
  try {
    transformed_point_stamped = tf_buffer_.transform(point, target_frame);
  } catch (tf2::TransformException & ex) {
    return false;
  }

  transformed_point = transformed_point_stamped.point;
  return true;
}

void LaneletCostmap::map_callback(const HADMapBin::SharedPtr msg)
{
  map_frame_id_ = msg->header.frame_id;
  map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map_);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(map_);
  roads_ = lanelet::utils::query::roadLanelets(all_lanelets);
}
}  // namespace lanelet_costmap
