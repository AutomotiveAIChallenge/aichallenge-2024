// Copyright 2024 Booars
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "stanley_control/stanley_control.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace stanley_control{

    using motion_utils::findNearestIndex;

    StanleyControl::StanleyControl(): Node("stanley_control"), tf_buffer(this->get_clock()), tf_listener(tf_buffer),
    speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 2.14)),
    external_target_vel_(declare_parameter<float>("external_target_vel", 10.0)),
    k_gain(declare_parameter<float>("k_gain", 2.0)),
    k_gain_slow(declare_parameter<float>("k_gain_slow", 1.0)){
        pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1.0);
        pub_marker_ = create_publisher<Marker>("debug/forward_point", 1);

        pub_angle_ = create_publisher<Float64>("output/angle", 1);

        sub_kinematics_ = create_subscription<Odometry>(
            "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
        sub_trajectory_ = create_subscription<Trajectory>(
            "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

        
        external_target_vel_=10.0;
        speed_proportional_gain_=1.0;
        using namespace std::literals::chrono_literals;
        timer_ = rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&StanleyControl::onTimer, this));

        // dynamic reconfigure
        auto parameter_change_cb = std::bind(&StanleyControl::parameter_callback, this, std::placeholders::_1);
        reset_param_handler_ = StanleyControl::add_on_set_parameters_callback(parameter_change_cb);
    }

    AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp){
        AckermannControlCommand cmd;
        cmd.stamp = stamp;
        cmd.longitudinal.stamp = stamp;
        cmd.longitudinal.speed = 0.0;
        cmd.longitudinal.acceleration = 0.0;
        cmd.lateral.stamp = stamp;
        cmd.lateral.steering_tire_angle = 0.0;
        return cmd;
    }

    void StanleyControl::onTimer(){
        // check data
        if (!subscribeMessageAvailable()) {
            return;
        }

        // create zero command
        AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

        // Check if goal is "reached"
        size_t closet_traj_point_idx = findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
        if ((closet_traj_point_idx == trajectory_->points.size() - 1) || (trajectory_->points.size() <= 5)) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
        }
        // Otherwise (main control)
        else {
            // get closest trajectory point from current position
            TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

            // longitudinal control
            double target_longitudinal_vel = external_target_vel_;
            double current_longitudinal_vel = odometry_->twist.twist.linear.x;

            cmd.longitudinal.speed = target_longitudinal_vel;
            cmd.longitudinal.acceleration =
            speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

            // calc lateral control
            //// calculate track yaw relative to the vehicle
            
            //// calculate track yaw angle and publish it 
            if (trajectory_->points.size() - (closet_traj_point_idx+1) <= 3 ){
               cmd.lateral.steering_tire_angle=0.0; 
            }
            else{
                // obtain closest point and next point on the trajectory
                size_t next_closest_traj_point_idx = closet_traj_point_idx+1;
                TrajectoryPoint next_closet_traj_point = trajectory_->points.at(next_closest_traj_point_idx);

                // calculate track "yaw" angle
                //// Trajectory vector
                double x_track = next_closet_traj_point.pose.position.x - closet_traj_point.pose.position.x;
                double y_track = next_closet_traj_point.pose.position.y - closet_traj_point.pose.position.y;

                double norm_track = sqrt(pow(x_track,2) + pow(y_track,2));

                //// vehicle heading vector in "map" frame
                //// obtain this by transforming constant vector in "base_link" frame to "map" frame
                Vector3Stamped vector_fixed, vector_vehicle;
                double norm_vehicle=1.0;
                {
                    vector_fixed.header.frame_id = "base_link";
                    vector_fixed.vector.x = 1.0;
                    vector_fixed.vector.y = 0.0;
                    vector_fixed.vector.z = 0.0;
                }
                try{
                    auto transform = tf_buffer.lookupTransform("map","base_link", tf2::TimePointZero);
                    tf2::doTransform(vector_fixed, vector_vehicle, transform);
                    norm_vehicle = sqrt(pow(vector_vehicle.vector.x, 2) + pow(vector_vehicle.vector.y, 2));
                }
                catch (tf2::TransformException &ex){
                    RCLCPP_ERROR(get_logger(), "Could not obtain transform from map to base_link: %s", ex.what());
                }

                // calculate the cosine of two vectors, from the inner product
                // use outer product to calculate signed angle (if v1xv2 >=0, then v2 is "to the left" of v1 )
                double ip_vector = x_track*vector_vehicle.vector.x + y_track*vector_vehicle.vector.y;
                double cos_vector = ip_vector / (norm_track * norm_vehicle);
                double angle = acos(cos_vector);

                double xp_vector = x_track*vector_vehicle.vector.y - y_track*vector_vehicle.vector.x;
                if(xp_vector>=0) angle*=-1; 

                // calculate positional error from nearest trajectory point

                error_distance = sqrt(pow(closet_traj_point.pose.position.x-odometry_->pose.pose.position.x,2)+pow(closet_traj_point.pose.position.y-odometry_->pose.pose.position.y,2));
                if (
                    (closet_traj_point.pose.position.x-odometry_->pose.pose.position.x)*vector_vehicle.vector.y 
                    - (closet_traj_point.pose.position.y-odometry_->pose.pose.position.y)*vector_vehicle.vector.x >= 0){
                    error_distance*=-1;
                }
                track_yaw = atan2(k_gain*error_distance, odometry_->twist.twist.linear.x+k_gain_slow);
                Float64 pub_angle = Float64();
                pub_angle.data = angle;
                pub_angle_->publish(pub_angle);
                cmd.lateral.steering_tire_angle=angle+track_yaw;
            }
        }
        pub_cmd_->publish(cmd);
    }

    bool StanleyControl::subscribeMessageAvailable(){
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
    rcl_interfaces::msg::SetParametersResult StanleyControl::parameter_callback(const std::vector<rclcpp::Parameter> &parameters){
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &parameter : parameters) {
            if (parameter.get_name() == "k_gain") {
                k_gain = parameter.as_double();
                RCLCPP_INFO(StanleyControl::get_logger(), "k_gain changed to %f", k_gain);
            } else if (parameter.get_name() == "k_gain_slow") {
                k_gain_slow = parameter.as_double();
                RCLCPP_INFO(StanleyControl::get_logger(), "k_gain_slow changed to %f", k_gain_slow);
            } else if (parameter.get_name() == "external_target_vel_") {
                external_target_vel_ = parameter.as_double();
                RCLCPP_INFO(StanleyControl::get_logger(), "external_target_vel changed to %f", external_target_vel_);
            }
            else if (parameter.get_name() == "speed_proportional_gain_") {
                speed_proportional_gain_ = parameter.as_double();
                RCLCPP_INFO(StanleyControl::get_logger(), "speed_proportional_gain changed to %f", speed_proportional_gain_);
            } 
        }
        return result;
    }
}  // namespace stanley_control

int main(int argc, char const * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stanley_control::StanleyControl>());
    rclcpp::shutdown();
    return 0;
}
