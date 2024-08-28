#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tier4_planning_msgs/msg/scenario.hpp"

class ParkingNode : public rclcpp::Node
{
public:
  ParkingNode() : Node("parking_node"), current_section_(0)
  {
    // Subscriber
    condition_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/condition", 10, std::bind(&ParkingNode::condition_callback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/status", 10, std::bind(&ParkingNode::status_callback, this, std::placeholders::_1));

    // Publisher
    scenario_pub_ = this->create_publisher<tier4_planning_msgs::msg::Scenario>("/scenario", 10);
  }

private:
  void condition_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    auto scenario_msg = tier4_planning_msgs::msg::Scenario();

    // 条件に基づいてシナリオを選択
    if (msg->data >= 1000 && current_section_ == 8)
    {
      scenario_msg.current_scenario = tier4_planning_msgs::msg::Scenario::PARKING;
      scenario_msg.activating_scenarios.push_back(tier4_planning_msgs::msg::Scenario::PARKING);
    }
    else
    {
      scenario_msg.current_scenario = tier4_planning_msgs::msg::Scenario::LANEDRIVING;
      scenario_msg.activating_scenarios.push_back(tier4_planning_msgs::msg::Scenario::LANEDRIVING);
    }

    scenario_pub_->publish(scenario_msg);
  }

  void status_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // data[3] にセクションがあると仮定
    if (msg->data.size() > 3)
    {
      current_section_ = msg->data[3];
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr condition_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr status_sub_;
  rclcpp::Publisher<tier4_planning_msgs::msg::Scenario>::SharedPtr scenario_pub_;

  float current_section_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}
