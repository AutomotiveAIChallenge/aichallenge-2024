#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>

class TrajectorySelector : public rclcpp::Node
{
public:
  TrajectorySelector() : Node("trajectory_selector")
  {
    scenario_subscriber_ = this->create_subscription<tier4_planning_msgs::msg::Scenario>(
      "/scenario", 10, std::bind(&TrajectorySelector::scenarioCallback, this, std::placeholders::_1));

    trajectory1_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "/trajectory_pit", 10, std::bind(&TrajectorySelector::trajectory1Callback, this, std::placeholders::_1));

    trajectory2_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "/trajectory_lane", 10, std::bind(&TrajectorySelector::trajectory2Callback, this, std::placeholders::_1));

    selected_trajectory_publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "/selected_trajectory", 10);
  }

private:
  void scenarioCallback(const tier4_planning_msgs::msg::Scenario::SharedPtr msg)
  {
    current_scenario_ = msg->current_scenario;
    publishSelectedTrajectory();
  }

  void trajectory1Callback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    trajectory1_ = msg;
    publishSelectedTrajectory();
  }

  void trajectory2Callback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    trajectory2_ = msg;
    publishSelectedTrajectory();
  }

  void publishSelectedTrajectory()
  {
    if (current_scenario_.empty()) {
      // シナリオが設定されていない場合は何もしない
      return;
    }

    autoware_auto_planning_msgs::msg::Trajectory::SharedPtr selected_trajectory;

    if (current_scenario_ == "Parking" && trajectory1_) {
      selected_trajectory = trajectory1_;
    } else if (current_scenario_ == "LaneDriving" && trajectory2_) {
      selected_trajectory = trajectory2_;
    } else if (!trajectory1_ && trajectory2_) {
      selected_trajectory = trajectory2_;
      RCLCPP_WARN(this->get_logger(), "Only trajectory_lane is available, publishing it.");
    } else if (trajectory1_ && !trajectory2_) {
      selected_trajectory = trajectory1_;
      RCLCPP_WARN(this->get_logger(), "Only trajectory_pit is available, publishing it.");
    } else {
      RCLCPP_WARN(this->get_logger(), "No valid trajectory available for the current scenario.");
      return;
    }

    selected_trajectory_publisher_->publish(*selected_trajectory);
  }

  rclcpp::Subscription<tier4_planning_msgs::msg::Scenario>::SharedPtr scenario_subscriber_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory1_subscriber_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory2_subscriber_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr selected_trajectory_publisher_;

  std::string current_scenario_;
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr trajectory1_;
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr trajectory2_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectorySelector>());
  rclcpp::shutdown();
  return 0;
}