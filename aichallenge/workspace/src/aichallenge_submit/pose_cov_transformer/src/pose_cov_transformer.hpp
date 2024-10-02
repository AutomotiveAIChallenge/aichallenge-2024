#ifndef POSE_COV_TRANSFORMER_HPP_
#define POSE_COV_TRANSFORMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace pose_cov_transformer
{

class Pose_cov_transformer : public rclcpp::Node
{

public:
    Pose_cov_transformer();

private:
    // Publish
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gnss_pose_cov_;

    // Subscription
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_cov_;
    void on_gnss_pose_cov(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string convert_frame_id_;
};

}

#endif