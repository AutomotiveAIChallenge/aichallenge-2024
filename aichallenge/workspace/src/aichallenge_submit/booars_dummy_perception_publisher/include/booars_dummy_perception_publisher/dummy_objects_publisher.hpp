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

#ifndef BOOARS_DUMMY_PERCEPTION_PUBLISHER__DUMMY_OBJECTS_PUBLISHER_HPP_
#define BOOARS_DUMMY_PERCEPTION_PUBLISHER__DUMMY_OBJECTS_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>

namespace booars_dummy_perception_publisher
{
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using PredictedObjects = autoware_auto_perception_msgs::msg::PredictedObjects;

class DummyObjectsPublisher : public rclcpp::Node
{
public:
  explicit DummyObjectsPublisher(const rclcpp::NodeOptions & options);

private:
  void objects_callback(const Float64MultiArray::SharedPtr msg);

  rclcpp::Subscription<Float64MultiArray>::SharedPtr objects_sub_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr objects_pub_;

  std::string map_frame_id_;
};
}  // namespace booars_dummy_perception_publisher

#endif  // BOOARS_DUMMY_PERCEPTION_PUBLISHER__DUMMY_OBJECTS_PUBLISHER_HPP_