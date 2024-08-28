#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tier4_planning_msgs/msg/scenario.hpp"

class ParkingNode : public rclcpp::Node
{
public:
  ParkingNode() : Node("parking_node")
  {
    // Subscriber
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "input_topic", 10, std::bind(&ParkingNode::int32_callback, this, std::placeholders::_1));

    // Publisher
    pub_ = this->create_publisher<tier4_planning_msgs::msg::Scenario>("output_topic", 10);
  }

private:
  void int32_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    auto scenario_msg = tier4_planning_msgs::msg::Scenario();

    if (msg->data >= 0)
    {
      scenario_msg.current_scenario = tier4_planning_msgs::msg::Scenario::PARKING;
      scenario_msg.activating_scenarios.push_back(tier4_planning_msgs::msg::Scenario::PARKING);
    }
    else
    {
      scenario_msg.current_scenario = tier4_planning_msgs::msg::Scenario::LANEDRIVING;
      scenario_msg.activating_scenarios.push_back(tier4_planning_msgs::msg::Scenario::LANEDRIVING);
    }

    pub_->publish(scenario_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Publisher<tier4_planning_msgs::msg::Scenario>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}
