#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2_ros/transform_listener.h>

class ImuGnssPoser : public rclcpp::Node
{

public:
    ImuGnssPoser() : Node("imu_gnss_poser"), tf2_listener_(tf2_buffer_)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/imu_gnss_poser/pose_with_covariance", qos);
        pub_initial_pose_3d_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/initial_pose3d", qos);
        sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/localization/twist_with_covariance", qos,
            std::bind(&ImuGnssPoser::twist_callback, this, std::placeholders::_1));
        sub_gnss_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss2/pose_with_covariance", qos,
            std::bind(&ImuGnssPoser::gnss_callback, this, std::placeholders::_1));
        sub_gnss2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss/pose_with_covariance", qos,
            std::bind(&ImuGnssPoser::gnss2_callback, this, std::placeholders::_1));
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensing/imu/imu_raw", qos,
            std::bind(&ImuGnssPoser::imu_callback, this, std::placeholders::_1));
    }

private:
    std::optional<geometry_msgs::msg::TransformStamped> get_transform(const std::string& target_frame) {
        try {
          return tf2_buffer_.lookupTransform(target_frame, "base_link", tf2::TimePointZero);
        } catch (tf2::TransformException & ex) {
          RCLCPP_WARN_STREAM_THROTTLE(
            this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(), ex.what());
          return std::nullopt;
        }
    }

    void gnss_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // msg->pose.pose.orientation.x = imu_msg_.orientation.x;
        // msg->pose.pose.orientation.y = imu_msg_.orientation.y;
        // msg->pose.pose.orientation.z = imu_msg_.orientation.z;
        // msg->pose.pose.orientation.w = imu_msg_.orientation.w;

        const auto base_to_gnss1 = get_transform("gnss_link");
        const auto base_to_gnss2 = get_transform("gnss2_link");

        // if (!last_gnss2_msg_.has_value()) {
        //   RCLCPP_WARN(this->get_logger(), "gnss2 is not received yet");
        // } else if(base_to_gnss1.has_value() && base_to_gnss2.has_value()) {
        //   const double base_link_to_antenna_dx = base_to_gnss1->transform.translation.x - base_to_gnss2->transform.translation.x;
        //   const double base_link_to_antenna_dy = base_to_gnss1->transform.translation.y - base_to_gnss2->transform.translation.y;
        //   const double base_link_to_antenna = std::atan2(base_link_to_antenna_dy, base_link_to_antenna_dx);

        //   const double map_to_antenna_dx = last_gnss2_msg_->pose.pose.position.x - msg->pose.pose.position.x;
        //   const double map_to_antenna_dy = last_gnss2_msg_->pose.pose.position.y - msg->pose.pose.position.y;
        //   const double map_to_antenna = std::atan2(map_to_antenna_dy, map_to_antenna_dx);

        //   const double yaw = map_to_antenna - base_link_to_antenna;

        //   msg->pose.pose.orientation.x = 0.0;
        //   msg->pose.pose.orientation.y = 0.0;
        //   msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
        //   msg->pose.pose.orientation.w = std::cos(yaw / 2.0);
        //   RCLCPP_INFO_STREAM_THROTTLE(
        //     this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(), "dual antenna heading: " << yaw);
        // } else {
        //   RCLCPP_WARN(this->get_logger(), "TF for gnss1 or gnss2 is not received yet");
        // }

        msg->pose.covariance[7 * 0] = 0.1;
        msg->pose.covariance[7 * 1] = 0.1;
        msg->pose.covariance[7 * 2] = 0.1;
        msg->pose.covariance[7 * 3] = 0.01;
        msg->pose.covariance[7 * 4] = 0.01;
        msg->pose.covariance[7 * 5] = 0.1;
        pub_pose_->publish(*msg);
        if (!is_ekf_initialized_)
            pub_initial_pose_3d_->publish(*msg);
    }

    void gnss2_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        last_gnss2_msg_ = *msg;
    }

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg_ = *msg;
    }

    void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        is_ekf_initialized_ = true;
    }

    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> last_gnss2_msg_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_3d_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss2_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    sensor_msgs::msg::Imu imu_msg_;
    bool is_ekf_initialized_ = {false};

    tf2::BufferCore tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuGnssPoser>());
    rclcpp::shutdown();
    return 0;
}
