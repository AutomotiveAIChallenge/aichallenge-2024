// Copyright 2024 Fool Stuck Engineers
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

#include "multi_layered_costmap/multi_layered_costmap.hpp"

namespace multi_layered_costmap
{
void MultiLayeredCostmap::add_costmap_layer(CostmapBase::SharedPtr costmap_layer)
{
  costmap_layers_.push_back(costmap_layer);
}

bool MultiLayeredCostmap::is_ready()
{
  for (const auto & costmap_layer : costmap_layers_) {
    if (!costmap_layer->is_ready()) return false;
  }
  return true;
}

bool MultiLayeredCostmap::is_occupied(const geometry_msgs::msg::PointStamped & point)
{
  for (const auto & costmap_layer : costmap_layers_) {
    if (!costmap_layer->is_occupied(point)) continue;
    return true;
  }
  return false;
}
}  // namespace multi_layered_costmap
