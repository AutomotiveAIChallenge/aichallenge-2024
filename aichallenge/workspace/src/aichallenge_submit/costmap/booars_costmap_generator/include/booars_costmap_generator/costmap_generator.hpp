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

#ifndef BOOARS_COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define BOOARS_COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include <booars_costmap_utils/booars_costmap_utils.hpp>
#include <multi_layered_costmap/multi_layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace booars_costmap_generator
{

using MultiLayeredCostmap = multi_layered_costmap::MultiLayeredCostmap;

class CostmapGenerator : public rclcpp::Node
{
public:
  explicit CostmapGenerator(const rclcpp::NodeOptions & options);

private:
  MultiLayeredCostmap::SharedPtr multi_layered_costmap_;
};
};  // namespace booars_costmap_generator

#endif  // BOOARS_COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
