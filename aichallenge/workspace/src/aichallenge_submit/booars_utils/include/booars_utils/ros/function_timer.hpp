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

#ifndef BOOARS_UTILS__ROS__FUNCTION_TIMER_HPP_
#define BOOARS_UTILS__ROS__FUNCTION_TIMER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>

namespace booars_utils::ros {

class FunctionTimer {
 private:
  rclcpp::TimerBase::SharedPtr timer_;

 public:
  using SharedPtr = std::shared_ptr<FunctionTimer>;

  static SharedPtr create_function_timer(rclcpp::Node* node,
                                         const double update_rate_hz,
                                         std::function<void()> callback) {
    return std::make_shared<FunctionTimer>(node, update_rate_hz, callback);
  }

  explicit FunctionTimer(rclcpp::Node* node, const double update_rate_hz,
                         std::function<void()> callback) {
    const double dt = 1.0 / update_rate_hz;

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(dt));
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(callback)>>(
        node->get_clock(), period, std::move(callback),
        node->get_node_base_interface()->get_context());
    node->get_node_timers_interface()->add_timer(timer_, nullptr);
  }
};

}  // namespace booars_utils::ros

#endif  // BOOARS_UTILS__ROS__FUNCTION_TIMER_HPP_
