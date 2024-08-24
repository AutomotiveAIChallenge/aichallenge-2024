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

#ifndef PREDICTED_OBJECT_COSTMAP__PREDICTED_OBJECT_COSTMAP_HPP_
#define PREDICTED_OBJECT_COSTMAP__PREDICTED_OBJECT_COSTMAP_HPP_

#include <multi_layered_costmap/costmap_base.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace predicted_object_costmap
{

using PredictedObject = autoware_auto_perception_msgs::msg::PredictedObject;
using PredictedObjects = autoware_auto_perception_msgs::msg::PredictedObjects;

class PredictedObjectCostmap : public multi_layered_costmap::CostmapBase
{
public:
  using SharedPtr = std::shared_ptr<PredictedObjectCostmap>;
  explicit PredictedObjectCostmap(const rclcpp::Node::SharedPtr & node);

  bool is_ready() override;
  bool is_occupied(const geometry_msgs::msg::PointStamped & point) override;

private:
  bool try_transform_point(
    const geometry_msgs::msg::PointStamped & point, geometry_msgs::msg::Point & transformed_point,
    const std::string & target_frame);
  bool is_intersected(const PredictedObject & object, const geometry_msgs::msg::Point & point);

  void objects_callback(const PredictedObjects::SharedPtr msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;

  PredictedObjects::SharedPtr objects_;
  double distance_threshold_;
};
}  // namespace predicted_object_costmap

#endif  // PREDICTED_OBJECT_COSTMAP__PREDICTED_OBJECT_COSTMAP_HPP_
