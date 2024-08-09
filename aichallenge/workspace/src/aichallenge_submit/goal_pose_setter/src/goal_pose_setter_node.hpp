// Copyright 2024 TIER IV, Inc.
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

#ifndef GOAL_POSE_SETTER_NODE_
#define GOAL_POSE_SETTER_NODE_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>



class GoalPosePublisher : public rclcpp::Node
{
public:
    GoalPosePublisher();

private:
    void on_timer();
    void route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ekf_trigger_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pit_position_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pit_condition_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pit_stop_time_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;
    bool stop_initializing_pose_ = false;
    bool stop_streaming_goal_pose_ = false;
    bool half_goal_pose_published_ = false;
    bool is_started_ = false;
    int delay_count_ = 0;
    float goal_range_;
    geometry_msgs::msg::Pose goal_position_;
    geometry_msgs::msg::Pose half_goal_position_;

    geometry_msgs::msg::Pose pit_position_;
    int pit_condition_;
    float pit_stop_time_;
    bool pit_in_flag_ = false;

    bool enable_pit_ = true;
    int pit_in_threshold_ = 1000;
};

#endif  // GOAL_POSE_SETTER_NODE_
