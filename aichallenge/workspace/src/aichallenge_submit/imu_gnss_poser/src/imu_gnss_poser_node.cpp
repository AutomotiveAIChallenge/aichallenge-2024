#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuGnssPoser : public rclcpp::Node
{

public:
    ImuGnssPoser() : Node("imu_gnss_poser")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/imu_gnss_poser/pose_with_covariance", qos);
        pub_initial_pose_3d_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/initial_pose3d", qos);
        sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/localization/twist_with_covariance", qos,
            std::bind(&ImuGnssPoser::twist_callback, this, std::placeholders::_1));
        sub_gnss_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss/pose_with_covariance", qos,
            std::bind(&ImuGnssPoser::gnss_callback, this, std::placeholders::_1));
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensing/imu/imu_raw", qos,
            std::bind(&ImuGnssPoser::imu_callback, this, std::placeholders::_1));
        
        specific_cov_ = this->declare_parameter<bool>("specific_covariance");
        posi_0_cov_ = this->declare_parameter<float>("position_0_cov");
        posi_1_cov_ = this->declare_parameter<float>("position_1_cov");
        posi_2_cov_ = this->declare_parameter<float>("position_2_cov");
        ori_0_cov_ = this->declare_parameter<float>("orientation_0_cov");
        ori_1_cov_ = this->declare_parameter<float>("orientation_1_cov");
        ori_2_cov_ = this->declare_parameter<float>("orientation_2_cov");
    }

private:

    void gnss_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

        // this covariance means orientation is not reliable
        if (specific_cov_){
            msg->pose.covariance[7*0] = posi_0_cov_;
            msg->pose.covariance[7*1] = posi_1_cov_;
            msg->pose.covariance[7*2] = posi_2_cov_;
            msg->pose.covariance[7*3] = ori_0_cov_;
            msg->pose.covariance[7*4] = ori_1_cov_;
            msg->pose.covariance[7*5] = ori_2_cov_;
        }
        // Only Simulation
        // insert imu if orientation is nan or empty
        if (std::isnan(msg->pose.pose.orientation.x) ||
            std::isnan(msg->pose.pose.orientation.y) ||
            std::isnan(msg->pose.pose.orientation.z) ||
            std::isnan(msg->pose.pose.orientation.w) ||
            (msg->pose.pose.orientation.x == 0 &&
             msg->pose.pose.orientation.y == 0 &&
             msg->pose.pose.orientation.z == 0 &&
             msg->pose.pose.orientation.w == 0))
        {
            msg->pose.pose.orientation.x = imu_msg_.orientation.x;
            msg->pose.pose.orientation.y = imu_msg_.orientation.y;
            msg->pose.pose.orientation.z = imu_msg_.orientation.z;
            msg->pose.pose.orientation.w = imu_msg_.orientation.w;
        }
        pub_pose_->publish(*msg);
        if (!is_ekf_initialized_)
            pub_initial_pose_3d_->publish(*msg);
    }

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg_ = *msg;
    }

    void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr)
    {
        is_ekf_initialized_ = true;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_3d_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    sensor_msgs::msg::Imu imu_msg_;
    bool is_ekf_initialized_ = {false};
    float posi_0_cov_;
    float posi_1_cov_;
    float posi_2_cov_;
    float ori_0_cov_;
    float ori_1_cov_;
    float ori_2_cov_;
    bool specific_cov_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuGnssPoser>());
    rclcpp::shutdown();
    return 0;
}
