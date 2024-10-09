#include "ackermann_control_publisher/ackermann_control_publisher.hpp"

namespace ackermann_control_publisher{
    AckermannControlPublisher::AckermannControlPublisher(): Node("ackermann_control_publisher"){

        pub_cmd_ = create_publisher<AckermannControlCommand>("output/ackermann_command", 1);

        sub_longitudinal_ = create_subscription<Float64>(
            "input/longitudinal", 1, [this](const Float64::SharedPtr msg) { longitudinal_acc_ = msg->data; });
        sub_lateral_ = create_subscription<Float64>(
            "input/lateral", 1, [this](const Float64::SharedPtr msg) { lateral_tire_angle_ = msg->data; });

        using namespace std::literals::chrono_literals;
        timer_ =
            rclcpp::create_timer(this, get_clock(), 5ms, std::bind(&AckermannControlPublisher::onTimer, this)); // 200Hz

    }

    void AckermannControlPublisher::onTimer(){
        // check data
        if (!subscribeMessageAvailable()) {
            return;
        }
        rclcpp::Time stamp = get_clock()->now();
        
        AckermannControlCommand cmd = AckermannControlCommand();
        cmd.stamp = stamp;
        cmd.longitudinal.stamp = stamp;
        cmd.lateral.stamp = stamp;

        cmd.longitudinal.acceleration = longitudinal_acc_;
        cmd.lateral.steering_tire_angle = lateral_tire_angle_;

        pub_cmd_->publish(cmd);
    }

    bool AckermannControlPublisher::subscribeMessageAvailable(){
        if (!longitudinal_acc_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
            return false;
        }
        if (!lateral_tire_angle_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
            return false;
        }
        return true;
    }
}

int main(int argc, char const * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ackermann_control_publisher::AckermannControlPublisher>());
  rclcpp::shutdown();
  return 0;
}