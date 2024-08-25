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

#include "booars_costmap_generator/costmap_generator.hpp"

namespace booars_costmap_generator
{
CostmapGenerator::CostmapGenerator(const rclcpp::NodeOptions & options)
: Node("costmap_generator", options)
{
  multi_layered_costmap_ =
    booars_costmap_utils::create_multi_layered_costmap(*this, "multi_layered_costmap");
}
}  // namespace booars_costmap_generator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(booars_costmap_generator::CostmapGenerator)
