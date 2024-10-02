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

#include "cached_lanelet_costmap/cached_lanelet_costmap.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace cached_lanelet_costmap
{
CachedLaneletCostmap::CachedLaneletCostmap(rclcpp::Node & node, const std::string & layer_namespace)
: tf_buffer_(node.get_clock()), tf_listener_(tf_buffer_)
{
  std::string map_topic = node.declare_parameter(layer_namespace + ".map_topic", "~/input/map");
  map_sub_ = node.create_subscription<HADMapBin>(
    map_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&CachedLaneletCostmap::map_callback, this, std::placeholders::_1));

  std::string costmap_topic =
    node.declare_parameter(layer_namespace + ".costmap_topic", "~/output/costmap");
  costmap_pub_ = node.create_publisher<OccupancyGrid>(
    costmap_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  // Declare cached costmap
  {
    double cached_costmap_min_x =
      node.declare_parameter(layer_namespace + ".cached_costmap.min_x", 0.0);
    double cached_costmap_min_y =
      node.declare_parameter(layer_namespace + ".cached_costmap.min_y", 0.0);
    double cached_costmap_max_x =
      node.declare_parameter(layer_namespace + ".cached_costmap.max_x", 0.0);
    double cached_costmap_max_y =
      node.declare_parameter(layer_namespace + ".cached_costmap.max_y", 0.0);
    double cached_costmap_resolution =
      node.declare_parameter(layer_namespace + ".cached_costmap.resolution", 0.0);

    cached_costmap_parameters_ = OccupancyGridParameters::create_parameters(
      cached_costmap_max_x - cached_costmap_min_x, cached_costmap_max_y - cached_costmap_min_y,
      cached_costmap_resolution);
    cached_costmap_origin_x_ = (cached_costmap_min_x + cached_costmap_max_x) * 0.5;
    cached_costmap_origin_y_ = (cached_costmap_min_y + cached_costmap_max_y) * 0.5;

    cached_costmap_ =
      booars_utils::nav::occupancy_grid_utils::create_occupancy_grid(cached_costmap_parameters_);

    geometry_msgs::msg::Vector3 origin;
    origin.x = cached_costmap_origin_x_;
    origin.y = cached_costmap_origin_y_;
    origin.z = 0.0;
    booars_utils::nav::occupancy_grid_utils::update_origin(
      cached_costmap_, cached_costmap_parameters_, origin);
  }

  // Declare other parameters
  {
    double inflation_radius = node.declare_parameter(layer_namespace + ".inflation_radius", 0.0);
    inflation_radius_index_ = std::ceil(inflation_radius / cached_costmap_parameters_->resolution());
  }
}

bool CachedLaneletCostmap::is_ready()
{
  return is_map_ready_;
}

bool CachedLaneletCostmap::is_occupied(const geometry_msgs::msg::PointStamped & point)
{
  if (!is_ready()) return true;

  geometry_msgs::msg::Point transformed_point;
  if (!try_transform_point(point, transformed_point, cached_costmap_->header.frame_id)) return true;

  tier4_autoware_utils::Point2d point2d(
    transformed_point.x - cached_costmap_origin_x_, transformed_point.y - cached_costmap_origin_y_);
  if (
    point2d[0] < -cached_costmap_parameters_->width_2() ||
    point2d[0] > cached_costmap_parameters_->width_2() ||
    point2d[1] < -cached_costmap_parameters_->height_2() ||
    point2d[1] > cached_costmap_parameters_->height_2()) {
    return true;
  }
  int index =
    booars_utils::nav::occupancy_grid_utils::point_to_index(cached_costmap_parameters_, point2d);
  if (index < 0 || index >= cached_costmap_parameters_->grid_num()) return true;
  return cached_costmap_->data[index] > 0;
}

bool CachedLaneletCostmap::try_transform_point(
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

void CachedLaneletCostmap::create_cached_costmap(
  const std::string & map_frame_id, const lanelet::ConstLanelets & roads_lanelets)
{
  std::vector<int> unoccupied_indices; 

  // Create costmap
  cached_costmap_->header.frame_id = map_frame_id;
  for (int i = 0; i < cached_costmap_parameters_->grid_num(); i++) {
    tier4_autoware_utils::Point2d point2d =
      booars_utils::nav::occupancy_grid_utils::index_to_point(cached_costmap_parameters_, i);
    point2d[0] += cached_costmap_origin_x_;
    point2d[1] += cached_costmap_origin_y_;
    cached_costmap_->data[i] = 100;
    for (const auto & road_lanelet : roads_lanelets) {
      if (!lanelet::geometry::within(point2d, road_lanelet.polygon2d().basicPolygon())) continue;
      cached_costmap_->data[i] = 0;
      unoccupied_indices.push_back(i);
      break;
    }
  }

  // Inflate costmap
  if (inflation_radius_index_ > 0) {
    int inflation_radius_index_sqr = inflation_radius_index_ * inflation_radius_index_;
    for (const auto & unoccupied_index : unoccupied_indices)
    {
      bool occupied = false;
      for (int i = -inflation_radius_index_; i <= inflation_radius_index_; i++) {
        for (int j = -inflation_radius_index_; j <= inflation_radius_index_; j++) {
          if (i*i + j*j > inflation_radius_index_sqr) continue;
          int index = unoccupied_index + i * cached_costmap_parameters_->grid_width() + j;
          if (index < 0 || index >= cached_costmap_parameters_->grid_num()) continue;
          if(cached_costmap_->data[index] != 100) continue;
          cached_costmap_->data[unoccupied_index] = 50;
          occupied = true;
          break;
        }
        if(occupied) break;
      }
    }
  }

  // Publish costmap
  costmap_pub_->publish(*cached_costmap_);

  is_map_ready_ = true;
}

void CachedLaneletCostmap::map_callback(const HADMapBin::SharedPtr msg)
{
  lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(map);
  const lanelet::ConstLanelets roads_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  create_cached_costmap(msg->header.frame_id, roads_lanelets);
}
}  // namespace cached_lanelet_costmap
