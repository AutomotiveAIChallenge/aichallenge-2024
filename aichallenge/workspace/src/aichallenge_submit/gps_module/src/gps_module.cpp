#include "gps_module/gps_module.hpp"

PoseComparisonNode::PoseComparisonNode() : Node("pose_comparison_node")
{
    // パラメータの宣言とデフォルト値
    this->declare_parameter<bool>("debug", false);
    debug_ = this->get_parameter("debug").as_bool();
    scale_pos_cov_x_ = this->declare_parameter<float>("scale_position_covariance_x");
    scale_pos_cov_y_ = this->declare_parameter<float>("scale_position_covariance_y");
    scale_pos_cov_z_ = this->declare_parameter<float>("scale_position_covariance_z");
    scale_ori_cov_0_ = this->declare_parameter<float>("scale_orientation_covariance_0");
    scale_ori_cov_1_ = this->declare_parameter<float>("scale_orientation_covariance_1");
    scale_ori_cov_2_ = this->declare_parameter<float>("scale_orientation_covariance_2");
    // original, csv, diff, only_cov
    mode_ = this->declare_parameter<std::string>("mode");

    // Use Path Yaw Culculation
    csv_path_ =this->declare_parameter<std::string>("csv_path");
    look_ahead_index_ = this->declare_parameter<int>("look_ahead_index");
    margin_ = this->declare_parameter<int>("margin");
    // If use_node_ is true, load csv file
    if (mode_ == "csv"){
        load_csv(csv_path_);
        index_ = 0;
        flag_ = false;
        set_trajectory_srv_ = this->create_service<csv_path_changer_msgs::srv::SetTrajectory>(
            "/set_trajectory_orientation",
            std::bind(&PoseComparisonNode::handle_trajectory, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    // サブスクリプションの設定
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "~/input/pose_with_covariance", 10, std::bind(&PoseComparisonNode::pose_callback, this, std::placeholders::_1));
    // PoseWithCovarianceStampedメッセージ用のパブリッシャー
    publisher_pose_with_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/pose_with_covariance", 10);
    // Poseメッセージ用のパブリッシャー
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/output/pose", 10);

}


void PoseComparisonNode::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // モードがoriginalの場合はPoseWithCovarianceStampedをそのままPublish
    if (mode_ == "original"|| mode_ == "pass_through")
    {
        auto pose_cov_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*msg);
        if (mode_ == "pass_through"){
            pose_cov_msg->pose.covariance[0] = msg->pose.covariance[0] * scale_pos_cov_x_;
            pose_cov_msg->pose.covariance[7] = msg->pose.covariance[7] * scale_pos_cov_y_;
            pose_cov_msg->pose.covariance[14] = msg->pose.covariance[14] * scale_pos_cov_z_;
            pose_cov_msg->pose.covariance[21] = msg->pose.covariance[21] * scale_ori_cov_0_;
            pose_cov_msg->pose.covariance[28] = msg->pose.covariance[28] * scale_ori_cov_1_;
            pose_cov_msg->pose.covariance[35] = msg->pose.covariance[35] * scale_ori_cov_2_;
        }
        else if(mode_ == "original"){
            pose_cov_msg->pose.covariance[0] = 0.1;
            pose_cov_msg->pose.covariance[7] = 0.1;
            pose_cov_msg->pose.covariance[14] = 0.1;
            pose_cov_msg->pose.covariance[21] = 100000.0;
            pose_cov_msg->pose.covariance[28] = 100000.0;
            pose_cov_msg->pose.covariance[35] = 100000.0;
        }
        publisher_pose_with_cov_->publish(*pose_cov_msg);
        publish_pose(pose_cov_msg);
        return;
    }
    else if(mode_ == "diff" || mode_ == "csv"){
        // キューに新しいメッセージを追加し、2つを超える場合は古いメッセージを削除
        if (pose_queue_.size() >= 2) {
            pose_queue_.pop_front();
        }
        pose_queue_.push_back(msg);

        // キューに2つのメッセージが揃ったら比較を行う
        if (pose_queue_.size() == 2) {
            compare_poses(pose_queue_[0], pose_queue_[1]);
        }
    }
}

void PoseComparisonNode::compare_poses(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& old_pose,
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& new_pose)
{
    // 位置の変化を確認
    bool position_change = 
        old_pose->pose.pose.position.x != new_pose->pose.pose.position.x ||
        old_pose->pose.pose.position.y != new_pose->pose.pose.position.y ||
        old_pose->pose.pose.position.z != new_pose->pose.pose.position.z;

    // 向き（四元数）の変化を確認
    bool orientation_change = 
        old_pose->pose.pose.orientation.x != new_pose->pose.pose.orientation.x ||
        old_pose->pose.pose.orientation.y != new_pose->pose.pose.orientation.y ||
        old_pose->pose.pose.orientation.z != new_pose->pose.pose.orientation.z ||
        old_pose->pose.pose.orientation.w != new_pose->pose.pose.orientation.w;

    // 位置または向きが変化した場合にイベントをトリガー
    // 基本的に新しいPoseをPublish
    if (position_change || orientation_change) {
        if (debug_) 
            RCLCPP_INFO(this->get_logger(), "Pose has changed!");

        // Publish PoseWithCovarianceStamped
        auto new_pose_modify_cov = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*new_pose);

        if (mode_ == "csv"){
            double new_x = new_pose->pose.pose.position.x;
            double new_y = new_pose->pose.pose.position.y;

            double min_dist = std::numeric_limits<double>::infinity();
            int min_index = 0;
            int search_index = 0;
            if (flag_) search_index = index_ + margin_;
            else search_index = points_;
            for (int i = index_; i < search_index; i++) {
                int index = i % points_;
                double dist = std::sqrt(
                    std::pow(new_x - trajectory_[index].position.x, 2) +
                    std::pow(new_y - trajectory_[index].position.y, 2)
                );

                if (dist < min_dist) {
                    min_dist = dist;
                    min_index = i;
                }
            }
            index_ = min_index;
            flag_ = true;

            geometry_msgs::msg::Pose look_ahead_pose = trajectory_[(min_index + look_ahead_index_) % points_];

            double angle_radians = calculate_slope_radians(
                new_x, new_y, look_ahead_pose.position.x, look_ahead_pose.position.y
            );

            std::vector<double> ori_from_path;
            euler_to_quaternion(0, 0, angle_radians, ori_from_path);

            new_pose_modify_cov->pose.pose.orientation.x = ori_from_path[0];
            new_pose_modify_cov->pose.pose.orientation.y = ori_from_path[1];
            new_pose_modify_cov->pose.pose.orientation.z = ori_from_path[2];
            new_pose_modify_cov->pose.pose.orientation.w = ori_from_path[3];
        }

        // Set Pose Cov
        new_pose_modify_cov->pose.covariance[0] = new_pose->pose.covariance[0] * scale_pos_cov_x_;            
        new_pose_modify_cov->pose.covariance[7] = new_pose->pose.covariance[7] * scale_pos_cov_y_;
        new_pose_modify_cov->pose.covariance[14] = new_pose->pose.covariance[14] * scale_pos_cov_z_;
        // Orientation Cov, nature 0.1 0.1 1.0
        new_pose_modify_cov->pose.covariance[21] = new_pose->pose.covariance[21] * scale_ori_cov_0_;       
        new_pose_modify_cov->pose.covariance[28] = new_pose->pose.covariance[28] * scale_ori_cov_1_;
        new_pose_modify_cov->pose.covariance[35] = new_pose->pose.covariance[35] * scale_ori_cov_2_;

        publisher_pose_with_cov_->publish(*new_pose_modify_cov);
        // Publish new pose
        publish_pose(new_pose_modify_cov);

    } else {
        if (debug_) 
            RCLCPP_INFO(this->get_logger(), "Pose not changed!");
    }
}

void PoseComparisonNode::publish_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& pose)
{
    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_msg->header = pose->header;
    pose_msg->pose = pose->pose.pose;

    publisher_pose_->publish(*pose_msg);
}

void PoseComparisonNode::handle_trajectory( const std::shared_ptr<csv_path_changer_msgs::srv::SetTrajectory::Request> request,
  std::shared_ptr<csv_path_changer_msgs::srv::SetTrajectory::Response> response)
{
  write_csv(request->csv_path, request->points);
  load_csv(request->csv_path);
  response->success = true;
}

void PoseComparisonNode::load_csv(std::string csv_path)
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

      trajectory_.push_back(pose);
    }
    file.close();
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu points", trajectory_.size());
  points_ = trajectory_.size();
  flag_ = false;
  index_ = 0;
}

void PoseComparisonNode::write_csv(const std::string &csv_path, const std::vector<TrajectoryPoint> &points)
{
  std::ofstream file(csv_path);
  
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing: %s", csv_path.c_str());
    return;
  }
  
  // ヘッダーを書き込む（必要に応じて変更）
  file << "x,y,z,x_quat,y_quat,z_quat,w_quat,speed\n";
  
  // 各ポイントを書き込む
  for (const auto &point : points) {
    const auto &pose = point.pose;
    file << pose.position.x << ","
         << pose.position.y << ","
         << pose.position.z << ","
         << pose.orientation.x << ","
         << pose.orientation.y << ","
         << pose.orientation.z << ","
         << pose.orientation.w << ","
         << point.longitudinal_velocity_mps << "\n";
  }
  
  file.close();
  RCLCPP_INFO(this->get_logger(), "Successfully wrote points to CSV: %s", csv_path.c_str());
}

double PoseComparisonNode::calculate_slope_radians(double start_x, double start_y, double end_x, double end_y)
{
    double angle_radians = 0;
    if (start_x == end_x) {
        if ((end_y - start_y) > 0) {
        angle_radians = M_PI / 2;
        } else {
        angle_radians = M_PI * 3 / 2; 
        }
    } else {
        angle_radians = std::atan2(end_y - start_y, end_x - start_x);
    }

    // RCLCPP_INFO(logger_, "angle_radians: %f", angle_radians);
    return angle_radians;
}

void PoseComparisonNode::euler_to_quaternion(double phi, double theta, double psi, std::vector<double>& result)
{
    double phi_half = phi / 2;
    double theta_half = theta / 2;
    double psi_half = psi / 2;

    double cos_phi_half = std::cos(phi_half);
    double sin_phi_half = std::sin(phi_half);
    double cos_theta_half = std::cos(theta_half);
    double sin_theta_half = std::sin(theta_half);
    double cos_psi_half = std::cos(psi_half);
    double sin_psi_half = std::sin(psi_half);

    double w = cos_phi_half * cos_theta_half * cos_psi_half + sin_phi_half * sin_theta_half * sin_psi_half;
    double x = sin_phi_half * cos_theta_half * cos_psi_half - cos_phi_half * sin_theta_half * sin_psi_half;
    double y = cos_phi_half * sin_theta_half * cos_psi_half + sin_phi_half * cos_theta_half * sin_psi_half;
    double z = cos_phi_half * cos_theta_half * sin_psi_half - sin_phi_half * sin_theta_half * cos_psi_half;

    result.push_back(x);
    result.push_back(y);
    result.push_back(z);
    result.push_back(w);

    return;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseComparisonNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
