#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class SectionTimerNode : public rclcpp::Node
{
public:
  SectionTimerNode()
  : Node("section_timer_node"), is_started_(false), best_lap_time_(std::numeric_limits<float>::infinity())
  {
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<double>("radius", 5.0);

    debug_ = this->get_parameter("debug").as_bool();
    radius_ = this->get_parameter("radius").as_double();

    if (debug_) {
      RCLCPP_INFO(this->get_logger(), "Debug mode is ON. Checking if YAML parameters are loaded...");
    }

    // セクション情報のロード
    for (int i = 1; i <= 12; ++i) {
      std::string section_name = "sections.section_" + std::to_string(i);

      this->declare_parameter<std::vector<double>>(section_name + ".start", {0.0, 0.0});
      this->declare_parameter<std::vector<double>>(section_name + ".end", {0.0, 0.0});

      std::vector<double> start;
      std::vector<double> end;

      if (this->get_parameter(section_name + ".start", start) && this->get_parameter(section_name + ".end", end)) {
        sections_.emplace_back(std::make_pair(start, end));
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameters for section %d", i);
      }
    }

    // Odomトピックのサブスクライバ
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odom", 1, std::bind(&SectionTimerNode::odometry_callback, this, std::placeholders::_1));

    // Float32MultiArrayパブリッシャの作成
    time_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("~/output/section_times", 10);
    time_diff_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("~/output/section_time_diffs", 10);

    // セクションタイム配列とベストタイム差分配列を初期化（13要素: 12セクション + 1ラップタイム）
    section_times_.resize(sections_.size() + 1, 0.0);
    best_times_.resize(sections_.size(), std::numeric_limits<float>::infinity());
    best_times_diff_.resize(sections_.size() + 1, 0.0);  // 最初は全て0.0で初期化
  }

private:

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (!is_started_) {
      if (distance(x, y, sections_[current_section_].first[0], sections_[current_section_].first[1]) < radius_) {
        RCLCPP_INFO(this->get_logger(), "Started section %d timer", current_section_ + 1);
        start_time_ = this->now();

        if (current_section_ == 0) {
          lap_start_time_ = this->now();
        }

        is_started_ = true;
      }
    } else {
      if (distance(x, y, sections_[current_section_].second[0], sections_[current_section_].second[1]) < radius_) {
        auto end_time = this->now();
        auto duration = end_time - start_time_;
        RCLCPP_INFO(this->get_logger(), "Finished section %d. Time: %.2f seconds",
                    current_section_ + 1, duration.seconds());

        // セクションタイムを更新
        section_times_[current_section_] = duration.seconds();

        // ベストタイム差分を計算・更新
        if (duration.seconds() < best_times_[current_section_]) {
          best_times_diff_[current_section_] = duration.seconds() - best_times_[current_section_];  // 早くなった秒数をマイナス付きで
          best_times_[current_section_] = duration.seconds();
        } else {
          best_times_diff_[current_section_] = duration.seconds() - best_times_[current_section_];
        }

        // タイムとベストタイムの差をパブリッシュ
        publish_section_times();
        publish_time_diffs();

        is_started_ = false;
        current_section_++;
        if (current_section_ >= sections_.size()) {
          // 1ラップ終了時に合計タイムを計算して表示
          auto lap_end_time = this->now();
          auto lap_duration = lap_end_time - lap_start_time_;
          section_times_.back() = lap_duration.seconds();

          // ラップタイムのベストとの差を計算
          if (lap_duration.seconds() < best_lap_time_) {
            best_times_diff_.back() = lap_duration.seconds() - best_lap_time_;  // 早くなった秒数をマイナスで
            best_lap_time_ = lap_duration.seconds();
          } else {
            best_times_diff_.back() = lap_duration.seconds() - best_lap_time_;
          }

          publish_section_times();
          publish_time_diffs();

          current_section_ = 0;  // 最初のセクションに戻る
        }
      }
    }
  }

  // タイム差のパブリッシュ
  void publish_time_diffs()
  {
    std_msgs::msg::Float32MultiArray diff_msg;
    diff_msg.data = best_times_diff_;  // 差分の配列全体をパブリッシュ
    time_diff_publisher_->publish(diff_msg);
  }

  // セクションタイムのパブリッシュ
  void publish_section_times()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = section_times_;  // セクションタイムとラップタイムを含める
    time_publisher_->publish(msg);
  }

  double distance(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr time_publisher_;  // タイムパブリッシャ
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr time_diff_publisher_;  // タイム差パブリッシャ
  std::vector<std::pair<std::vector<double>, std::vector<double>>> sections_;
  std::vector<float> section_times_;  // セクションタイムとラップタイムを保持する配列
  std::vector<float> best_times_;  // 各セクションのベストタイムを保持する配列
  std::vector<float> best_times_diff_;  // ベストタイムとの差を保持する配列
  float best_lap_time_;  // ラップタイムのベストタイム
  rclcpp::Time start_time_;
  rclcpp::Time lap_start_time_;
  bool is_started_;
  int current_section_ = 0;
  double radius_;
  bool debug_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SectionTimerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
