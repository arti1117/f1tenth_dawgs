#pragma once

#ifndef LQR_PATH_FOLLOWER_NODE_HPP
#define LQR_PATH_FOLLOWER_NODE_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "lqr_path_follower/lqr_controller.hpp"

// Quaternion to yaw conversion
inline double quatToYaw(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

class LQRPathFollowerNode : public rclcpp::Node {
public:
    LQRPathFollowerNode();

private:
    // LQR controller
    std::unique_ptr<LQRController> lqr_controller_;

    // Parameters
    std::string odom_topic_;
    std::string drive_topic_;
    std::string path_topic_;
    std::string base_frame_;

    double path_timeout_;
    bool debug_mode_;
    double velocity_gain_;

    // Path data
    std::vector<LQRController::State> current_path_;
    std::vector<double> path_velocities_;
    rclcpp::Time last_path_time_;
    bool path_received_;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predicted_path_pub_;

    // Callbacks
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Visualization
    void visualizePredictedPath(
        const std::vector<LQRController::State>& predicted_states);

    // Utility
    void publishStopCommand();
};

#endif // LQR_PATH_FOLLOWER_NODE_HPP
