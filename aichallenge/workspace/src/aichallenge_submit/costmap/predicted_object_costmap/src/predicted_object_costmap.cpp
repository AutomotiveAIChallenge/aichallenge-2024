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

#include "predicted_object_costmap/predicted_object_costmap.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>

namespace predicted_object_costmap
{
PredictedObjectCostmap::PredictedObjectCostmap(
  rclcpp::Node & node, const std::string & layer_namespace)
: tf_buffer_(node.get_clock()), tf_listener_(tf_buffer_)
{
  std::string objects_topic =
    node.declare_parameter(layer_namespace + ".predicted_objects_topic", "~/input/objects");
  objects_sub_ = node.create_subscription<PredictedObjects>(
    objects_topic, 1,
    std::bind(&PredictedObjectCostmap::objects_callback, this, std::placeholders::_1));

  distance_threshold_ = node.declare_parameter(layer_namespace + ".distance_threshold", 0.0);
}

bool PredictedObjectCostmap::is_ready()
{
  return objects_ != nullptr;
}

bool PredictedObjectCostmap::is_occupied(const geometry_msgs::msg::PointStamped & point)
{
  if (!is_ready()) return true;

  geometry_msgs::msg::Point transformed_point;
  if (!try_transform_point(point, transformed_point, objects_->header.frame_id)) {
    return true;
  }

  for (const auto & object : objects_->objects) {
    if (!is_intersected(object, transformed_point)) continue;
    return true;
  }
  return false;
}

bool PredictedObjectCostmap::try_transform_point(
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

bool PredictedObjectCostmap::is_intersected(
  const PredictedObject & object, const geometry_msgs::msg::Point & point)
{
  if (object.shape.type != object.shape.CYLINDER) return false;

  const geometry_msgs::msg::Point center =
    object.kinematics.initial_pose_with_covariance.pose.position;
  const double radius =
    std::max(object.shape.dimensions.x, object.shape.dimensions.y) * 0.5 + distance_threshold_;
  const double sqr_radius = radius * radius;

  const double dx = point.x - center.x;
  const double dy = point.y - center.y;
  const double sqr_distance = dx * dx + dy * dy;

  return sqr_distance < sqr_radius;
}

void PredictedObjectCostmap::objects_callback(const PredictedObjects::SharedPtr msg)
{
  objects_ = msg;
}

}  // namespace predicted_object_costmap
