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
    RCLCPP_INFO(this->get_logger(), "============= pose cov transformer ==============");
    using std::placeholders::_1;

    // pub, subの初期化
    sub_gnss_pose_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance", 1, std::bind(&Pose_cov_transformer::on_gnss_pose_cov, this, _1));
    pub_gnss_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensing/gnss/base/pose_with_covariance", 1);
}

void Pose_cov_transformer::on_gnss_pose_cov(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    auto process_and_publish_gnss_cov = [this, msg]() {

        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
        transform_stamped = tf_buffer_.lookupTransform("base_link", "map", rclcpp::Time(0), rclcpp::Duration(1, 0));
        } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Could not transform from map to base_link: %s", ex.what());
        return;
        }
        auto pose_cov = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*msg);
        tf2::doTransform(*pose_cov, *pose_cov, transform_stamped);
        pose_cov->header.stamp = now();
        pose_cov->header.frame_id = "base_link";
        pose_cov->pose.pose.position.x += msg->pose.pose.position.x;
        pose_cov->pose.pose.position.y += msg->pose.pose.position.y;
        pose_cov->pose.pose.position.z = msg->pose.pose.position.z;
        pose_cov->pose.pose.orientation.x = msg->pose.pose.orientation.x;
        pose_cov->pose.pose.orientation.y = msg->pose.pose.orientation.y;
        pose_cov->pose.pose.orientation.z = msg->pose.pose.orientation.z;
        pose_cov->pose.pose.orientation.w = msg->pose.pose.orientation.w;
        
        pub_gnss_pose_cov_->publish(*pose_cov);
  };
  std::thread processing_thread(process_and_publish_gnss_cov);
  processing_thread.detach();
}

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_cov_transformer::Pose_cov_transformer>());
    rclcpp::shutdown();
    return 0;
}
