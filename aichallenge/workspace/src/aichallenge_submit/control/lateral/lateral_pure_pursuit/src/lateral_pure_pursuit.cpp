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

#include "lateral_pure_pursuit/lateral_pure_pursuit.hpp"
#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>

namespace lateral_pure_pursuit{

    using motion_utils::findNearestIndex;
    using tier4_autoware_utils::calcLateralDeviation;
    using tier4_autoware_utils::calcYawDeviation;

    LateralPurePursuit::LateralPurePursuit(): Node("lateral_pure_pursuit"),
    // initialize parameters
    wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
    lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
    lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
    extra_steering_gain_(declare_parameter<float>("steering_tire_angle_gain",1.0)){

        pub_cmd_ = create_publisher<Float64>("output/steer_angle", 1);
        pub_marker_ = create_publisher<Marker>("debug/pursuit_lookahead2", 1);

        sub_kinematics_ = create_subscription<Odometry>(
            "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
        sub_trajectory_ = create_subscription<Trajectory>(
            "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

        using namespace std::literals::chrono_literals;
        timer_ =
            rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&LateralPurePursuit::onTimer, this));

        // dynamic reconfigure
        auto parameter_change_cb = std::bind(&LateralPurePursuit::parameter_callback, this, std::placeholders::_1);
        reset_param_handler_ = LateralPurePursuit::add_on_set_parameters_callback(parameter_change_cb);

        // steering gain
        // todo: make this configurable from dynamic reconfigure
    }

    void LateralPurePursuit::onTimer(){
        // check data
        if (!subscribeMessageAvailable()) {
            return;
        }

        size_t closet_traj_point_idx =
            findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

        Float64 cmd_msg = Float64();
        if (
            (closet_traj_point_idx == trajectory_->points.size() - 1) ||
            (trajectory_->points.size() <= 5)) {
            cmd_msg.data = 0.0;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
        } else {
            // get closest trajectory point from current position
            // double current_longitudinal_vel = odometry_->twist.twist.linear.x;
            double current_longitudinal_vel = 8.0;
            
            // calc lateral control
            //// calc lookahead distance
            double lookahead_distance = lookahead_gain_ * current_longitudinal_vel + lookahead_min_distance_;
            //// calc center coordinate of rear wheel
            ////// assuming odom is in the middle of the car
            double rear_x = odometry_->pose.pose.position.x -
                            wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
            double rear_y = odometry_->pose.pose.position.y -
                            wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
            //// search lookahead point
            // * todo: change this so that the actual distance on the track is considered
            auto lookahead_point_itr = std::find_if(
            trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
            [&](const TrajectoryPoint & point) {
                return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
                    lookahead_distance;
            });
            if (lookahead_point_itr == trajectory_->points.end()) {
            lookahead_point_itr = trajectory_->points.end() - 1;
            }
            double lookahead_point_x = lookahead_point_itr->pose.position.x;
            double lookahead_point_y = lookahead_point_itr->pose.position.y;
            
            // publish lookahead point marker
            {
                auto marker_msg = Marker();
                marker_msg.header.frame_id = "map";
                marker_msg.header.stamp = now();
                marker_msg.ns = "basic_shapes";
                marker_msg.id = 0;
                marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
                marker_msg.action = visualization_msgs::msg::Marker::ADD;
                marker_msg.pose.position.x = lookahead_point_x;
                marker_msg.pose.position.y = lookahead_point_y;
                marker_msg.pose.position.z = 80;
                marker_msg.pose.orientation.x = 0.0;
                marker_msg.pose.orientation.y = 0.0;
                marker_msg.pose.orientation.z = 0.0;
                marker_msg.pose.orientation.w = 1.0;
                marker_msg.scale.x = 3.0;
                marker_msg.scale.y = 3.0;
                marker_msg.scale.z = 3.0;
                marker_msg.color.r = 1.0f;
                marker_msg.color.g = 0.0f;
                marker_msg.color.b = 0.0f;
                marker_msg.color.a = 1.0;
                pub_marker_->publish(marker_msg);
            }
            // calc steering angle for lateral control
            double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                        tf2::getYaw(odometry_->pose.pose.orientation);
            cmd_msg.data =
            std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance) * extra_steering_gain_;
        }
        pub_cmd_->publish(cmd_msg);
    }

    bool LateralPurePursuit::subscribeMessageAvailable(){
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
    rcl_interfaces::msg::SetParametersResult LateralPurePursuit::parameter_callback(const std::vector<rclcpp::Parameter> &parameters){
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &parameter : parameters) {
            if (parameter.get_name() == "lookahead_gain") {
                lookahead_gain_ = parameter.as_double();
                RCLCPP_INFO(LateralPurePursuit::get_logger(), "lookahead_gain changed to %f", lookahead_gain_);
            } 
            else if (parameter.get_name() == "lookahead_min_distance") {
                lookahead_min_distance_ = parameter.as_double();
                RCLCPP_INFO(LateralPurePursuit::get_logger(), "lookahead_min_distance changed to %f", lookahead_min_distance_);
            } 
            else if (parameter.get_name() == "steering_tire_angle_gain") {
                extra_steering_gain_ = parameter.as_double();
                RCLCPP_INFO(LateralPurePursuit::get_logger(), "steering_tire_angle_gain changed to %f", extra_steering_gain_);
            }
        }
        return result;
    }
}  // namespace lateral_pure_pursuit



int main(int argc, char const * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lateral_pure_pursuit::LateralPurePursuit>());
  rclcpp::shutdown();
  return 0;
}
