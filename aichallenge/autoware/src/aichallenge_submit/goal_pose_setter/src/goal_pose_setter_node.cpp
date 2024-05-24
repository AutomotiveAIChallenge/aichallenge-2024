#include "goal_pose_setter_node.hpp"

GoalPosePublisher::GoalPosePublisher() : Node("goal_pose_publisher")
{
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    ekf_trigger_client_ = this->create_client<std_srvs::srv::SetBool>("/localization/trigger_node");
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", qos);
    route_state_subscriber_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
        "/planning/mission_planning/route_state",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        std::bind(&GoalPosePublisher::route_state_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(300),
        std::bind(&GoalPosePublisher::on_timer, this));

    this->declare_parameter("goal.position.x", 21920.2);
    this->declare_parameter("goal.position.y", 51741.1);
    this->declare_parameter("goal.position.z", 0.0);
    this->declare_parameter("goal.orientation.x", 0.0);
    this->declare_parameter("goal.orientation.y", 0.0);
    this->declare_parameter("goal.orientation.z", 0.645336);
    this->declare_parameter("goal.orientation.w", 0.763899);
}

void GoalPosePublisher::on_timer()
{
    if (delay_count_ <= 10) {
        ++delay_count_;
        return;
    }

    if (!stop_initializing_pose_) {
        if (ekf_trigger_client_->service_is_ready()) {
            const auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = true;
            ekf_trigger_client_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
            {
                stop_initializing_pose_ = future.get()->success;
                RCLCPP_INFO(this->get_logger(), "Complete localization trigger");
            });
            RCLCPP_INFO(this->get_logger(), "Call localization trigger");
        }
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

void GoalPosePublisher::route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
    if (msg->state >= autoware_adapi_v1_msgs::msg::RouteState::SET)
    {
        stop_streaming_goal_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "Stop streaming goal pose");
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
