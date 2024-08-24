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

#include "multi_layered_costmap_visualizer/multi_layered_costmap_visualizer.hpp"

namespace multi_layered_costmap_visualizer
{
MultiLayeredCostmapVisualizer::MultiLayeredCostmapVisualizer(const rclcpp::NodeOptions & options)
: Node("multi_layered_costmap_visualizer", options)
{
  multi_layered_costmap_ =
    booars_costmap_utils::create_multi_layered_costmap(*this, "multi_layered_costmap");
}
}  // namespace multi_layered_costmap_visualizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(multi_layered_costmap_visualizer::MultiLayeredCostmapVisualizer)
