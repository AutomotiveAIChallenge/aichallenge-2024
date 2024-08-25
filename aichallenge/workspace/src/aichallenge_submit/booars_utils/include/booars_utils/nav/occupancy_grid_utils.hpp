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

#ifndef BOOARS_UTILS__NAV__OCCUPANCY_GRID_UTILS_HPP_
#define BOOARS_UTILS__NAV__OCCUPANCY_GRID_UTILS_HPP_

#include "booars_utils/nav/occupancy_grid_parameters.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace booars_utils::nav::occupancy_grid_utils
{
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

OccupancyGridParameters::SharedPtr create_occupancy_grid_parameters(
  const OccupancyGrid::SharedPtr occupancy_grid)
{
  const double width = occupancy_grid->info.width * occupancy_grid->info.resolution;
  return OccupancyGridParameters::create_parameters(width, occupancy_grid->info.resolution);
}

OccupancyGrid::SharedPtr create_occupancy_grid(
  const OccupancyGridParameters::SharedPtr parameters, const int8_t value = 0)
{
  OccupancyGrid::SharedPtr occupancy_grid = std::make_shared<OccupancyGrid>();
  occupancy_grid->info.width = parameters->grid_width();
  occupancy_grid->info.height = parameters->grid_width();
  occupancy_grid->info.resolution = parameters->resolution();
  occupancy_grid->info.origin.position.x = -parameters->width_2();
  occupancy_grid->info.origin.position.y = -parameters->width_2();
  occupancy_grid->data.resize(parameters->grid_num(), value);
  return occupancy_grid;
}

void update_origin(
  OccupancyGrid::SharedPtr occupancy_grid, const OccupancyGridParameters::SharedPtr parameters,
  const geometry_msgs::msg::Vector3 & translation)
{
  occupancy_grid->info.origin.position.x = -parameters->width_2() + translation.x;
  occupancy_grid->info.origin.position.y = -parameters->width_2() + translation.y;
  occupancy_grid->info.origin.position.z = translation.z;
  occupancy_grid->info.origin.orientation.x = 0.0;
  occupancy_grid->info.origin.orientation.y = 0.0;
  occupancy_grid->info.origin.orientation.z = 0.0;
  occupancy_grid->info.origin.orientation.w = 1.0;
}

tier4_autoware_utils::Point2d index_to_point(
  const OccupancyGridParameters::SharedPtr parameters, const int index)
{
  const int index_x = index % parameters->grid_width();
  const int index_y = index / parameters->grid_width();

  return {
    index_x * parameters->resolution() - parameters->width_2(),
    index_y * parameters->resolution() - parameters->width_2()};
}

std::vector<tier4_autoware_utils::Point2d> get_index_to_point_table(
  const OccupancyGridParameters::SharedPtr parameters)
{
  std::vector<tier4_autoware_utils::Point2d> table;
  table.reserve(parameters->grid_num());
  for (int i = 0; i < parameters->grid_num(); ++i) {
    table.push_back(index_to_point(parameters, i));
  }
  return table;
}

}  // namespace booars_utils::nav::occupancy_grid_utils

#endif  // BOOARS_UTILS__NAV__OCCUPANCY_GRID_UTILS_HPP_
