#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <std_msgs/msg/float64.hpp>

class VehicleStateCalculator : public rclcpp::Node
{
public:
    VehicleStateCalculator() : Node("vehicle_state_calculator")
    {
        velocity_subscription_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10,
            std::bind(&VehicleStateCalculator::velocity_callback, this, std::placeholders::_1));

        steering_subscription_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", 10,
            std::bind(&VehicleStateCalculator::steering_callback, this, std::placeholders::_1));

        acceleration_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/vehicle/status/acceleration", 10);
        angular_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/vehicle/status/angular_velocity", 10);

        last_velocity_ = 0.0;
        last_steering_angle_ = 0.0;
        last_velocity_timestamp_ = this->now();
        last_steering_timestamp_ = this->now();
    }

private:
    void velocity_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double current_velocity = msg->longitudinal_velocity;
        
        double dt = (current_time - last_velocity_timestamp_).seconds();
        
        if (dt > 0) {
            double acceleration = (current_velocity - last_velocity_) / dt;
            
            auto acceleration_msg = std_msgs::msg::Float64();
            acceleration_msg.data = acceleration;
            acceleration_publisher_->publish(acceleration_msg);
        }
        
        last_velocity_ = current_velocity;
        last_velocity_timestamp_ = current_time;
    }

    void steering_callback(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double current_steering_angle = msg->steering_tire_angle;
        
        double dt = (current_time - last_steering_timestamp_).seconds();
        
        if (dt > 0) {
            // Convert steering angle to angular velocity (degree)
            double angular_velocity = 57.29 * (current_steering_angle - last_steering_angle_) / dt;
            
            auto angular_velocity_msg = std_msgs::msg::Float64();
            angular_velocity_msg.data = angular_velocity;
            angular_velocity_publisher_->publish(angular_velocity_msg);
        }
        
        last_steering_angle_ = current_steering_angle;
        last_steering_timestamp_ = current_time;
    }

    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr acceleration_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angular_velocity_publisher_;
    double last_velocity_;
    double last_steering_angle_;
    rclcpp::Time last_velocity_timestamp_;
    rclcpp::Time last_steering_timestamp_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleStateCalculator>());
    rclcpp::shutdown();
    return 0;
}