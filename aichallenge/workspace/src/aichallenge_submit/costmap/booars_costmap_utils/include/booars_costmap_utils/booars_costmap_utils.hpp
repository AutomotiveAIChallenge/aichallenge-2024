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

#ifndef BOOARS_COSTMAP_UTILS__BOOARS_COSTMAP_UTILS_HPP_
#define BOOARS_COSTMAP_UTILS__BOOARS_COSTMAP_UTILS_HPP_

#include <cached_lanelet_costmap/cached_lanelet_costmap.hpp>
#include <lanelet_costmap/lanelet_costmap.hpp>
#include <multi_layered_costmap/multi_layered_costmap.hpp>
#include <predicted_object_costmap/predicted_object_costmap.hpp>

namespace booars_costmap_utils
{
using CachedLaneletCostmap = cached_lanelet_costmap::CachedLaneletCostmap;
using LaneletCostmap = lanelet_costmap::LaneletCostmap;
using MultiLayeredCostmap = multi_layered_costmap::MultiLayeredCostmap;
using PredictedObjectCostmap = predicted_object_costmap::PredictedObjectCostmap;

MultiLayeredCostmap::SharedPtr create_multi_layered_costmap(
  rclcpp::Node & node, const std::string & costmap_namespace)
{
  auto costmap = std::make_shared<MultiLayeredCostmap>();

  auto layers = node.declare_parameter(costmap_namespace + ".layers", std::vector<std::string>());

  for (const auto & layer : layers) {
    std::string layer_namespace = costmap_namespace + "." + layer;
    auto type = node.declare_parameter(layer_namespace + ".type", std::string());

    if (type == "cached_lanelet") {
      auto cached_lanelet_costmap = CachedLaneletCostmap::create_costmap(node, layer_namespace);
      costmap->add_costmap_layer(cached_lanelet_costmap);
    } else if (type == "lanelet") {
      auto lanelet_costmap = LaneletCostmap::create_costmap(node, layer_namespace);
      costmap->add_costmap_layer(lanelet_costmap);
    } else if (type == "predicted_object") {
      auto predicted_object_costmap = PredictedObjectCostmap::create_costmap(node, layer_namespace);
      costmap->add_costmap_layer(predicted_object_costmap);
    } else {
      RCLCPP_ERROR(node.get_logger(), "Unknown layer type: %s", type.c_str());
    }
  }

  return costmap;
}

}  // namespace booars_costmap_utils

#endif
