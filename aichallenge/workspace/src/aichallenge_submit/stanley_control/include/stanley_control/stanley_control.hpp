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
#ifndef STANLEY_HPP_
#define STANLEY_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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
#include <cmath>

namespace stanley_control{
    using autoware_auto_control_msgs::msg::AckermannControlCommand;
    using autoware_auto_planning_msgs::msg::Trajectory;
    using autoware_auto_planning_msgs::msg::TrajectoryPoint;
    using geometry_msgs::msg::Pose;
    using geometry_msgs::msg::Twist;
    using geometry_msgs::msg::Vector3Stamped;
    using visualization_msgs::msg::Marker;
    using std_msgs::msg::Float64;
    using nav_msgs::msg::Odometry;

    class StanleyControl : public rclcpp::Node {
        public:
            explicit StanleyControl();
            
            // subscribers
            rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
            rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;

            // tf2 listeners
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener;
            
            // publishers
            rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
            rclcpp::Publisher<Float64>::SharedPtr pub_angle_;
            rclcpp::Publisher<Marker>::SharedPtr pub_marker_;
            
            // timer for control
            rclcpp::TimerBase::SharedPtr timer_;

            // updated by subscribers
            Trajectory::SharedPtr trajectory_;
            Odometry::SharedPtr odometry_;

            // stanley parameters
            // speed control
            double speed_proportional_gain_;
            double external_target_vel_;
            // stanley parameters
            double k_gain;
            double k_gain_slow;

        private:
            void onTimer();
            bool subscribeMessageAvailable();
            
            
            // control variables
            double error_distance;
            double track_yaw;
            OnSetParametersCallbackHandle::SharedPtr reset_param_handler_;
            rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);
    };
} // namespace stanley_control


#endif //STANLEY_HPP_