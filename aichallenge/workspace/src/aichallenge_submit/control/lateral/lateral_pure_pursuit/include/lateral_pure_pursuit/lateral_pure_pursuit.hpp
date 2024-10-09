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
#ifndef LATERAL_PURE_PURSUIT_HPP_
#define LATERAL_PURE_PURSUIT_HPP_

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cmath>

namespace lateral_pure_pursuit{
    using autoware_auto_planning_msgs::msg::Trajectory;
    using autoware_auto_planning_msgs::msg::TrajectoryPoint;
    using geometry_msgs::msg::Pose;
    using geometry_msgs::msg::Twist;
    using geometry_msgs::msg::Vector3Stamped;
    using visualization_msgs::msg::Marker;
    using std_msgs::msg::Float64;
    using nav_msgs::msg::Odometry;

    class LateralPurePursuit : public rclcpp::Node {
        public:
            explicit LateralPurePursuit();
            
            // subscribers
            rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
            rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
            
            // publishers
            rclcpp::Publisher<Float64>::SharedPtr pub_cmd_;
            rclcpp::Publisher<Marker>::SharedPtr pub_marker_;
            
            // timer for control
            rclcpp::TimerBase::SharedPtr timer_;

            // updated by subscribers
            Trajectory::SharedPtr trajectory_;
            Odometry::SharedPtr odometry_;

            
            

        private:
            void onTimer();
            bool subscribeMessageAvailable();
            
            //parameters
            double wheel_base_;              // Distance between front and rear axle, needed for geometric control
            double lookahead_gain_;          // Determines how the lookahead distance grows in relationship with speed
            double lookahead_min_distance_;  // Minimum lookahead distance, any lookahead distance will be built on top of this
            double extra_steering_gain_;     // Duct tape fix constant to account for lack of steering motor power

            OnSetParametersCallbackHandle::SharedPtr reset_param_handler_;
            rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);
    };
} // namespace lateral_pure_pursuit


#endif //LATERAL_PURE_PURSUIT_HPP_