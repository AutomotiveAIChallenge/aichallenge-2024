#ifndef GPS_MODULE__TRUE_GPS_MODULE_HPP_
#define GPS_MODULE__TRUE_GPS_MODULE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath> 
#include "csv_path_changer_msgs/srv/set_trajectory.hpp"

class PoseComparisonNode : public rclcpp::Node {
public:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
  PoseComparisonNode();

private:
  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_pose_with_cov_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;

  // Subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // Client

  // Server
  rclcpp::Service<csv_path_changer_msgs::srv::SetTrajectory>::SharedPtr set_trajectory_srv_;
  void handle_trajectory(const std::shared_ptr<csv_path_changer_msgs::srv::SetTrajectory::Request> request,
                        std::shared_ptr<csv_path_changer_msgs::srv::SetTrajectory::Response> response);

  // Param
  bool debug_;
  float scale_pos_cov_x_;
  float scale_pos_cov_y_;
  float scale_pos_cov_z_;
  float scale_ori_cov_0_;
  float scale_ori_cov_1_;
  float scale_ori_cov_2_;
  std::string csv_path_;
  int look_ahead_index_;
  int margin_;
  bool use_node_;

  // Function
  void compare_poses(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& old_pose,
                       const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& new_pose);
  void load_csv(std::string csv_path);
  double calculate_slope_radians(double start_x, double start_y, double end_x, double end_y);
  void euler_to_quaternion(double phi, double theta, double psi, std::vector<double>& result);
  void write_csv(const std::string &csv_path, const std::vector<TrajectoryPoint> &points);

  // Variable
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_queue_;
  std::vector<geometry_msgs::msg::Pose> trajectory_;
  int points_;
  int index_;
  bool flag_;
};

#endif