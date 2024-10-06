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
    sub_kinematic_state_ = create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1, std::bind(&Pose_transformer::on_kinematic_state, this, _1));
    pub_kinematic_state_ = create_publisher<nav_msgs::msg::Odometry>("/localization/base_link/kinematic_state", 1);
    sub_imu_raw_ = create_subscription<sensor_msgs::msg::Imu>("/sensing/imu/imu_raw", 1, std::bind(&Pose_transformer::on_imu_data, this, _1));
    pub_imu_raw_ = create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/base_link/imu_raw", 1);
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

void Pose_transformer::on_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        // mapフレームの値をbase_linkフレームの値に変換するための変換行列の算出
        transform_stamped = tf_buffer_.lookupTransform(convert_frame_id_, msg->header.frame_id, rclcpp::Time(0), rclcpp::Duration(1, 0));
    }
    catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Could not find transformation: %s", ex.what());
    }
    auto odom_transformed = std::make_shared<nav_msgs::msg::Odometry>();
    // msgの値を変換してodom_transformedに格納
    tf2::doTransform(msg->pose.pose, odom_transformed->pose.pose, transform_stamped);

    odom_transformed->header.stamp = msg->header.stamp;
    odom_transformed->header.frame_id = convert_frame_id_;
    odom_transformed->child_frame_id = convert_frame_id_;
    // child_frame_idは元々base_linkで、twistはchild_frame_idで指定された座標フレームで表されるので、twistは変換していない
    odom_transformed->twist = msg->twist;
    
    pub_kinematic_state_->publish(*odom_transformed);
}

void Pose_transformer::on_imu_data(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform(convert_frame_id_, msg->header.frame_id, rclcpp::Time(0), rclcpp::Duration(1, 0));
        
    }
    catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Could not find transformation: %s", ex.what());
        return;
    }

    auto imu_transformed = std::make_shared<sensor_msgs::msg::Imu>();

    tf2::doTransform(msg->orientation, imu_transformed->orientation, transform_stamped);
    tf2::doTransform(msg->angular_velocity, imu_transformed->angular_velocity, transform_stamped);
    tf2::doTransform(msg->linear_acceleration, imu_transformed->linear_acceleration, transform_stamped);

    imu_transformed->header.stamp = msg->header.stamp;
    imu_transformed->header.frame_id = convert_frame_id_;

    imu_transformed->orientation_covariance = msg->orientation_covariance;
    imu_transformed->angular_velocity_covariance = msg->angular_velocity_covariance;
    imu_transformed->linear_acceleration_covariance = msg->linear_acceleration_covariance;

    pub_imu_raw_->publish(*imu_transformed);
    
}
}// namespace pose_transformer
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_transformer::Pose_transformer>());
    rclcpp::shutdown();
    return 0;
}
