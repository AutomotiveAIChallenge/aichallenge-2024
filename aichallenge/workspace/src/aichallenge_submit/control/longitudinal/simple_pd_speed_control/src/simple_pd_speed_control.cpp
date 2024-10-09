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
#include "simple_pd_speed_control/simple_pd_speed_control.hpp"

#include <motion_utils/motion_utils.hpp>

#include <algorithm>

namespace simple_speed_pd_control{

    using motion_utils::findNearestIndex;

    SimpleSpeedPDControl::SimpleSpeedPDControl()
    : Node("simple_speed_pd_control"), speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)){
        pub_cmd_ = create_publisher<Float64>("output/target_acc", 1);

        sub_kinematics_ = create_subscription<Odometry>(
            "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
        sub_trajectory_ = create_subscription<Trajectory>(
            "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

        using namespace std::literals::chrono_literals;
        timer_ =
            rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimpleSpeedPDControl::onTimer, this)); // 100Hz

        // dynamic reconfigure
        auto parameter_change_cb = std::bind(&SimpleSpeedPDControl::parameter_callback, this, std::placeholders::_1);
        reset_param_handler_ = SimpleSpeedPDControl::add_on_set_parameters_callback(parameter_change_cb);
    }

    void SimpleSpeedPDControl::onTimer(){
        // check data
        if (!subscribeMessageAvailable()) {
            return;
        }

        size_t closet_traj_point_idx = findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

        // get closest trajectory point from current position
        TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

        // calc longitudinal speed and acceleration
        double target_longitudinal_vel = closet_traj_point.longitudinal_velocity_mps;
        double current_longitudinal_vel = odometry_->twist.twist.linear.x;
        Float64 msg = Float64();
        msg.data = speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

        pub_cmd_->publish(msg);
    }

    bool SimpleSpeedPDControl::subscribeMessageAvailable(){
        if (!odometry_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
            return false;
        }
        if (!trajectory_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
            return false;
        }
        return true;
    }
    rcl_interfaces::msg::SetParametersResult SimpleSpeedPDControl::parameter_callback(const std::vector<rclcpp::Parameter> &parameters){
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto &parameter : parameters) {
        if (parameter.get_name() == "speed_proportional_gain") {
        speed_proportional_gain_ = parameter.as_double();
        RCLCPP_INFO(SimpleSpeedPDControl::get_logger(), "speed_proportional_gain changed to %f", speed_proportional_gain_);
        }
    }
    return result;
    }
}  // namespace simple_speed_pd_control



int main(int argc, char const * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_speed_pd_control::SimpleSpeedPDControl>());
  rclcpp::shutdown();
  return 0;
}
