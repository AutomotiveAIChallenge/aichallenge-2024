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

#include "booars_dummy_perception_publisher/dummy_objects_publisher.hpp"

namespace booars_dummy_perception_publisher
{
DummyObjectsPublisher::DummyObjectsPublisher(const rclcpp::NodeOptions & options)
: Node("dummy_objects_publisher", options)
{
  map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");

  objects_sub_ = create_subscription<Float64MultiArray>(
    "~/input/objects", 10,
    std::bind(&DummyObjectsPublisher::objects_callback, this, std::placeholders::_1));
  objects_pub_ = create_publisher<PredictedObjects>("~/output/objects", 10);
}

void DummyObjectsPublisher::objects_callback(const Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() % 4 != 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid message size");
    return;
  }
  int object_count = msg->data.size() / 4;

  PredictedObjects objects_msg;

  for (int i = 0; i < object_count; i++) {
    autoware_auto_perception_msgs::msg::PredictedObject object;

    object.object_id.uuid[0] = i;

    object.kinematics.initial_pose_with_covariance.pose.position.x = msg->data[i * 4 + 0];
    object.kinematics.initial_pose_with_covariance.pose.position.y = msg->data[i * 4 + 1];
    object.kinematics.initial_pose_with_covariance.pose.position.z = msg->data[i * 4 + 2];
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;

    object.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
    const double size = msg->data[i * 4 + 3] * 2.0;
    object.shape.dimensions.x = size;
    object.shape.dimensions.y = size;
    object.shape.dimensions.z = 1.0;

    objects_msg.objects.push_back(object);
  }

  objects_msg.header.frame_id = map_frame_id_;
  objects_msg.header.stamp = now();
  objects_pub_->publish(objects_msg);
}
}  // namespace booars_dummy_perception_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(booars_dummy_perception_publisher::DummyObjectsPublisher)