#include <rclcpp/rclcpp.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>

namespace param_controller
{

class ParamController : public rclcpp::Node
{
public:
  ParamController()
  : Node("param_controller"), steering_value_(0.0)
  {
    publisher_ = this->create_publisher<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
      "/control/command/actuation_cmd", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ParamController::timer_callback, this));

    // Parameter declaration
    this->declare_parameter("steering_angle", 0.0);

    // Parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ParamController::param_callback, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {
    auto message = tier4_vehicle_msgs::msg::ActuationCommandStamped();
    message.header.stamp = this->now();
    message.actuation.steer_cmd = steering_value_;
    publisher_->publish(message);
  }

  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "steering_angle") {
        steering_value_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Parameter updated: steering_angle = %.2f", steering_value_);
      }
    }

    return result;
  }

  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  double steering_value_;
};

} // namespace param_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<param_controller::ParamController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}