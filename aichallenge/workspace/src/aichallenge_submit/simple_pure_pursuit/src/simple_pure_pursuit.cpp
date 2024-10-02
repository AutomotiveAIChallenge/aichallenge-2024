#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0))
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  mkr_cmd_ = create_publisher<Marker>("debug/pursuit_lookahead", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&SimplePurePursuit::onTimer, this));

  // dynamic reconfigure
  auto parameter_change_cb = std::bind(&SimplePurePursuit::parameter_callback, this, std::placeholders::_1);
  reset_param_handler_ = SimplePurePursuit::add_on_set_parameters_callback(parameter_change_cb);
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  if (
    (closet_traj_point_idx == trajectory_->points.size() - 1) ||
    (trajectory_->points.size() <= 2)) {
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration = -10.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
  } else {
    // get closest trajectory point from current position
    TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

    // calc longitudinal speed and acceleration
    double target_longitudinal_vel =
      use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
    double current_longitudinal_vel = odometry_->twist.twist.linear.x;

    cmd.longitudinal.speed = target_longitudinal_vel;
    cmd.longitudinal.acceleration =
      speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

    // calc lateral control
    //// calc lookahead distance
    double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
    //// calc center coordinate of rear wheel
    double rear_x = odometry_->pose.pose.position.x -
                    wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
    double rear_y = odometry_->pose.pose.position.y -
                    wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
    //// search lookahead point
    auto lookahead_point_itr = std::find_if(
      trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
      [&](const TrajectoryPoint & point) {
        return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
               lookahead_distance;
      });
    if (lookahead_point_itr == trajectory_->points.end()) {
      lookahead_point_itr = trajectory_->points.end() - 1;
    }
    double lookahead_point_x = lookahead_point_itr->pose.position.x;
    double lookahead_point_y = lookahead_point_itr->pose.position.y;
    {
      // publish lookahead point marker
      auto marker_msg = Marker();
      marker_msg.header.frame_id = "map";
      marker_msg.header.stamp = now();
      marker_msg.ns = "basic_shapes";
      marker_msg.id = 0;
      marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
      marker_msg.pose.position.x = lookahead_point_x;
      marker_msg.pose.position.y = lookahead_point_y;
      marker_msg.pose.position.z = 80;
      marker_msg.pose.orientation.x = 0.0;
      marker_msg.pose.orientation.y = 0.0;
      marker_msg.pose.orientation.z = 0.0;
      marker_msg.pose.orientation.w = 1.0;
      marker_msg.scale.x = 3.0;
      marker_msg.scale.y = 3.0;
      marker_msg.scale.z = 3.0;
      marker_msg.color.r = 1.0f;
      marker_msg.color.g = 0.0f;
      marker_msg.color.b = 0.0f;
      marker_msg.color.a = 1.0;
      mkr_cmd_->publish(marker_msg);
    }
    // calc steering angle for lateral control
    double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                   tf2::getYaw(odometry_->pose.pose.orientation);
    cmd.lateral.steering_tire_angle =
      steering_tire_angle_gain_ * std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
  }
  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  return true;
}
rcl_interfaces::msg::SetParametersResult SimplePurePursuit::parameter_callback(const std::vector<rclcpp::Parameter> &parameters){
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (const auto &parameter : parameters) {
    if (parameter.get_name() == "lookahead_gain") {
      lookahead_gain_ = parameter.as_double();
      RCLCPP_INFO(SimplePurePursuit::get_logger(), "lookahead_gain changed to %f", lookahead_gain_);
    } else if (parameter.get_name() == "lookahead_min_distance") {
      lookahead_min_distance_ = parameter.as_double();
      RCLCPP_INFO(SimplePurePursuit::get_logger(), "lookahead_min_distance changed to %f", lookahead_min_distance_);
    } else if (parameter.get_name() == "external_target_vel") {
      external_target_vel_ = parameter.as_double();
      RCLCPP_INFO(SimplePurePursuit::get_logger(), "external_target_vel changed to %f", external_target_vel_);
    } 
  }
  return result;
}
}  // namespace simple_pure_pursuit



int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
