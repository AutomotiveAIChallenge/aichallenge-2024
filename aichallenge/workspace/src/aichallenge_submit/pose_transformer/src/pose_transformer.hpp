#ifndef POSE_COV_TRANSFORMER_HPP_
#define POSE_COV_TRANSFORMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <thread>
#include "tf2/utils.h"
#include <cmath>
#include <utility>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace pose_transformer
{

class Pose_transformer : public rclcpp::Node
{

public:
    Pose_transformer();

private:
    // Publish
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gnss_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_orig_gnss_pose_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_state_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_raw_;

    // Subscription
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_cov_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_raw_;
    void on_gnss_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
    void on_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void on_imu_data(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string convert_frame_id_;
    bool pub_gnss_pose_original_;
};

}

#endif