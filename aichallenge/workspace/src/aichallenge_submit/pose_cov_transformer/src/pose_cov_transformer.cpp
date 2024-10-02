#include "pose_cov_transformer.hpp"
#include "tf2/utils.h"
#include <cmath>
#include <utility>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace pose_cov_transformer
{

Pose_cov_transformer::Pose_cov_transformer() : Node("pose_with_trasnformer"),
tf_buffer_(get_clock()),
tf_listener_(tf_buffer_, this, false)
{
    tf_buffer_.setUsingDedicatedThread(true);
    using std::placeholders::_1;

    // pub, subの初期化
    sub_gnss_pose_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance", 1, std::bind(&Pose_cov_transformer::on_gnss_pose_cov, this, _1));
    pub_gnss_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensing/gnss/base_link/pose_with_covariance", 1);
    convert_frame_id_ = this->declare_parameter<std::string>("frame_id", "base_link");
}

void Pose_cov_transformer::on_gnss_pose_cov(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform(convert_frame_id_, msg->header.frame_id, rclcpp::Time(0), rclcpp::Duration(1, 0));
    } 
    catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Could not find transformation: %s", ex.what());
        return;
    }
    auto pose_cov = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    tf2::doTransform(*msg, *pose_cov, transform_stamped);
    pose_cov->header.stamp = msg->header.stamp;
    pose_cov->header.frame_id = convert_frame_id_;
    // pose_cov->pose.pose.position.x = msg->pose.pose.position.x;
    // pose_cov->pose.pose.position.y = msg->pose.pose.position.y;
    // pose_cov->pose.pose.position.z = msg->pose.pose.position.z;
    // pose_cov->pose.pose.orientation.x = msg->pose.pose.orientation.x;
    // pose_cov->pose.pose.orientation.y = msg->pose.pose.orientation.y;
    // pose_cov->pose.pose.orientation.z = msg->pose.pose.orientation.z;
    // pose_cov->pose.pose.orientation.w = msg->pose.pose.orientation.w;
    
    pub_gnss_pose_cov_->publish(*pose_cov);
}
}// namespace pose_cov_transformer
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_cov_transformer::Pose_cov_transformer>());
    rclcpp::shutdown();
    return 0;
}
