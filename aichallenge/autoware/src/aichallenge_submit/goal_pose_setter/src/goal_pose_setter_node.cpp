#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"

class GoalPosePublisher : public rclcpp::Node
{
    bool stop_streaming_goal_pose_ = false;

public:
    GoalPosePublisher() : Node("goal_pose_publisher")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", qos);
        route_state_subscriber_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
            "/planning/mission_planning/route_state",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
            std::bind(&GoalPosePublisher::route_state_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300),
            std::bind(&GoalPosePublisher::publish_goal_pose, this));

        this->declare_parameter("goal.position.x", 21920.2);
        this->declare_parameter("goal.position.y", 51741.1);
        this->declare_parameter("goal.position.z", 0.0);
        this->declare_parameter("goal.orientation.x", 0.0);
        this->declare_parameter("goal.orientation.y", 0.0);
        this->declare_parameter("goal.orientation.z", 0.645336);
        this->declare_parameter("goal.orientation.w", 0.763899);

        delay_count_ = 0;
    }

private:
    void publish_goal_pose()
    {
        if (delay_count_ <= 10) {
          ++delay_count_;
          return;
        }

        if (!stop_streaming_goal_pose_)
        {
            auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
            msg->header.stamp = this->get_clock()->now();
            msg->header.frame_id = "map";
            msg->pose.position.x = this->get_parameter("goal.position.x").as_double();
            msg->pose.position.y = this->get_parameter("goal.position.y").as_double();
            msg->pose.position.z = this->get_parameter("goal.position.z").as_double();
            msg->pose.orientation.x = this->get_parameter("goal.orientation.x").as_double();
            msg->pose.orientation.y = this->get_parameter("goal.orientation.y").as_double();
            msg->pose.orientation.z = this->get_parameter("goal.orientation.z").as_double();
            msg->pose.orientation.w = this->get_parameter("goal.orientation.w").as_double();

            goal_publisher_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Publishing goal pose");
        }
    }

    void route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
    {
        if (msg->state >= autoware_adapi_v1_msgs::msg::RouteState::SET)
        {
            stop_streaming_goal_pose_ = true;
            RCLCPP_INFO(this->get_logger(), "Stop streaming goal pose");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    // subscribe route state
    rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    int delay_count_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
