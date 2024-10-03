#include "pose_transformer.hpp"

namespace pose_transformer
{

Pose_transformer::Pose_transformer() : Node("pose_trasnformer"),
tf_buffer_(get_clock()),
tf_listener_(tf_buffer_, this, false)
{
    tf_buffer_.setUsingDedicatedThread(true);
    using std::placeholders::_1;

    // pub, subの初期化
    sub_gnss_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>("/sensing/gnss/pose", 1, std::bind(&Pose_transformer::on_gnss_pose, this, _1));
    pub_gnss_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/sensing/gnss/base_link/pose", 1);
    convert_frame_id_ = this->declare_parameter<std::string>("frame_id", "base_link");
}

void Pose_transformer::on_gnss_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform(convert_frame_id_, msg->header.frame_id, rclcpp::Time(0), rclcpp::Duration(1, 0));
    } 
    catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Could not find transformation: %s", ex.what());
        return;
    }
    auto pose_cov = std::make_shared<geometry_msgs::msg::PoseStamped>();
    tf2::doTransform(*msg, *pose_cov, transform_stamped);
    pose_cov->header.stamp = msg->header.stamp;
    pose_cov->header.frame_id = convert_frame_id_;
    
    pub_gnss_pose_->publish(*pose_cov);
}
}// namespace pose_transformer
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_transformer::Pose_transformer>());
    rclcpp::shutdown();
    return 0;
}
