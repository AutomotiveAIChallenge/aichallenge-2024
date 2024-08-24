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

#ifndef MULTI_LAYERED_COSTMAP__COSTMAP_BASE_HPP_
#define MULTI_LAYERED_COSTMAP__COSTMAP_BASE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <memory>

namespace multi_layered_costmap
{
class CostmapBase
{
public:
  using SharedPtr = std::shared_ptr<CostmapBase>;
  virtual bool is_ready() = 0;
  virtual bool is_occupied(const geometry_msgs::msg::PointStamped & point) = 0;
};
}  // namespace multi_layered_costmap

#endif  // MULTI_LAYERED_COSTMAP__COSTMAP_BASE_HPP_
