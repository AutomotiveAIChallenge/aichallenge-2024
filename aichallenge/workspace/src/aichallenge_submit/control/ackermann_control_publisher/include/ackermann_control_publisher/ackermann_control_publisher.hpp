#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <std_msgs/msg/float64.hpp>

namespace ackermann_control_publisher {
    using namespace std::literals::chrono_literals;
    using std_msgs::msg::Float64;
    using autoware_auto_control_msgs::msg::AckermannControlCommand;

    class AckermannControlPublisher : public rclcpp::Node {
        public:
            explicit AckermannControlPublisher();
            
            // subscribers
            rclcpp::Subscription<Float64>::SharedPtr sub_longitudinal_;
            rclcpp::Subscription<Float64>::SharedPtr sub_lateral_;
            
            // publishers
            rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
            
            // timer for control
            rclcpp::TimerBase::SharedPtr timer_;

        private:

            // control command
            double longitudinal_acc_; // 目標加速度 [m/s^2]
            double lateral_tire_angle_; // タイヤ舵角 [rad]

            void onTimer();
            bool subscribeMessageAvailable();
    };
} //namespace ackermann_control_publisher