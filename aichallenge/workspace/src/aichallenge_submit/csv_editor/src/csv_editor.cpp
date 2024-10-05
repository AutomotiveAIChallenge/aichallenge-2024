#include "csv_editor/csv_editor.hpp"
#include <fstream>
#include <sstream>
#include <string>

CsvEditor::CsvEditor() : Node("csv_editor")
{
  RCLCPP_INFO(this->get_logger(), "================ Csv Editor ==================");

  this->declare_parameter("base_path", std::string("base_path"));
  base_path_ = this->get_parameter("base_path").as_string();

  RCLCPP_INFO(this->get_logger(), "base_path: %s", base_path_.c_str());

  set_trajectory_client_ = this->create_client<csv_path_changer_msgs::srv::SetTrajectory>("/set_trajectory");

  while (!set_trajectory_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "path client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "wait for trajectory service to appear...");
  }

  load_csv(base_path_);
}

void CsvEditor::load_csv(std::string csv_path)
{
  RCLCPP_INFO(this->get_logger(), "--------------- Load CSV %s ---------------", csv_path.c_str());

  std::ifstream file(csv_path);
  std::string line;
  if (!file.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Failed to open CSV file");
  } else {
    RCLCPP_INFO(this->get_logger(), "Reading CSV file");
    std::getline(file, line);

    while (std::getline(file, line))
    {
      std::stringstream ss(line);
      std::string x, y, z, x_quat, y_quat, z_quat, w_quat, speed;
      std::getline(ss, x, ',');
      std::getline(ss, y, ',');
      std::getline(ss, z, ',');
      std::getline(ss, x_quat, ',');
      std::getline(ss, y_quat, ',');
      std::getline(ss, z_quat, ',');
      std::getline(ss, w_quat, ',');
      std::getline(ss, speed, ',');

      geometry_msgs::msg::Pose pose;
      pose.position.x = std::stof(x);
      pose.position.y = std::stof(y);
      pose.position.z = 43.1;
      pose.orientation.x = std::stof(x_quat);
      pose.orientation.y = std::stof(y_quat);
      pose.orientation.z = std::stof(z_quat);
      pose.orientation.w = std::stof(w_quat);

      TrajectoryPoint point;
      point.pose = pose;
      point.longitudinal_velocity_mps = std::stof(speed);
      points_.push_back(point);
    }
    file.close();
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu points", points_.size());

  set_trajectory_request();
}

void CsvEditor::set_trajectory_request() {
  auto request = std::make_shared<csv_path_changer_msgs::srv::SetTrajectory::Request>();

  request->csv_path = base_path_;
  request->points = points_;

  using ServiceResponseFuture = rclcpp::Client<csv_path_changer_msgs::srv::SetTrajectory>::SharedFuture;

  // レスポンス処理
  auto response_callback = [this](ServiceResponseFuture future) {
    if (future.get()->success) {  // success フィールドを確認
      RCLCPP_INFO(this->get_logger(), "Set Trajectory successfully.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to set Trajectory.");
    }
  };

  set_trajectory_client_->async_send_request(request, response_callback);
  RCLCPP_INFO(this->get_logger(), "Send Trajectory");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvEditor>());
  rclcpp::shutdown();
  return 0;
}
