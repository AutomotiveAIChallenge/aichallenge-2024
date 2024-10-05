#ifndef CSV_EDITOR__CSV_EDITOR_HPP_
#define CSV_EDITOR__CSV_EDITOR_HPP_

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "csv_path_changer_msgs/srv/set_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>

class CsvEditor : public rclcpp::Node {
  public:
    using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

    CsvEditor();

  private:
    // Publisher

    // Subscription

    // Client
    rclcpp::Client<csv_path_changer_msgs::srv::SetTrajectory>::SharedPtr set_trajectory_client_;
    void set_trajectory_request();

    // Server

    // Param
    std::string base_path_;

    // function
    void load_csv(std::string csv_path);

    // variable
    std::vector<TrajectoryPoint> points_;
};

#endif
