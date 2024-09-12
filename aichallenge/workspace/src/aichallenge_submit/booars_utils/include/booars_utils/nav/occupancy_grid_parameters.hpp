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

#ifndef BOOARS_UTILS__NAV__OCCUPANCY_GRID_PARAMETERS_HPP_
#define BOOARS_UTILS__NAV__OCCUPANCY_GRID_PARAMETERS_HPP_

#include <memory>

namespace booars_utils::nav
{
class OccupancyGridParameters
{
public:
  using SharedPtr = std::shared_ptr<OccupancyGridParameters>;

  static OccupancyGridParameters::SharedPtr create_parameters(
    const double width, const double height, const double resolution)
  {
    return std::make_shared<OccupancyGridParameters>(width, height, resolution);
  }

  explicit OccupancyGridParameters(const double width, const double height, const double resolution)
  {
    width_ = width;
    width_2_ = width / 2.0;
    height_ = height;
    height_2_ = height / 2.0;
    resolution_ = resolution;
    resolution_inv_ = 1.0 / resolution;
    grid_width_2_ = static_cast<int>(width * resolution_inv_) / 2;
    grid_width_ = grid_width_2_ * 2;
    grid_height_2_ = static_cast<int>(height * resolution_inv_) / 2;
    grid_height_ = grid_height_2_ * 2;
    grid_num_ = grid_width_ * grid_height_;
  }

  double width() const { return width_; }
  double width_2() const { return width_2_; }
  double height() const { return height_; }
  double height_2() const { return height_2_; }
  double resolution() const { return resolution_; }
  double resolution_inv() const { return resolution_inv_; }
  int grid_width_2() const { return grid_width_2_; }
  int grid_width() const { return grid_width_; }
  int grid_height_2() const { return grid_height_2_; }
  int grid_height() const { return grid_height_; }
  int grid_num() const { return grid_num_; }

private:
  double width_;
  double width_2_;
  double height_;
  double height_2_;
  double resolution_;
  double resolution_inv_;
  int grid_width_2_;
  int grid_width_;
  int grid_height_2_;
  int grid_height_;
  int grid_num_;
};

}  // namespace booars_utils::nav

#endif  // BOOARS_UTILS__NAV__OCCUPANCY_GRID_PARAMETERS_HPP_
