#ifndef POSE_COV_TRANSFORMER_HPP_
#define POSE_COV_TRANSFORMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include "tf2/utils.h"
#include <cmath>
#include <utility>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/odometry.hpp"

namespace pose_transformer
{

class Pose_transformer : public rclcpp::Node
{

public:
    Pose_transformer();

private:
    // Publish
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gnss_pose_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_state_;

    // Subscription
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_gnss_pose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_;
    void on_gnss_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
    void on_kinematic_state_(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string convert_frame_id_;
};

}

#endif