#ifndef COSTMAP_SERVER_COSTMAP_2D_HPP_
#define COSTMAP_SERVER_COSTMAP_2D_HPP_

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace costmap_server
{
class CostMap2D : public rclcpp::Node
{

public:
    CostMap2D();

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_2d_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_sub_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void object_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void send_map_request();
    void createInflationLayer(nav_msgs::msg::OccupancyGrid & map);
    void calculateInflation(
        nav_msgs::msg::OccupancyGrid & map, const uint32_t & map_x, const uint32_t & map_y);
    double calculateCost(double stochastic_variable, double inflation_radius);
    double normalizeCost(double max_cost, double cost);
    std::tuple<int, int, int, int> calculateIndex(
        double x_m, double y_m, double radius
    );

    nav_msgs::msg::OccupancyGrid map_;
    double inflation_radius_;
    double wall_width_;
    bool get_map_;
    double width_, height_;
    double origin_x_, origin_y_;
    double resolution_;

    int top_margin_, bottom_margin_, left_margin_, right_margin_;
    int color_;
};
}

#endif