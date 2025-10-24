#pragma once

#ifndef PATH_TRACKER_HPP
#define PATH_TRACKER_HPP

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"

struct PathPoint {
  double x;
  double y;
  double yaw;
  double v;  // desired speed at this point
};

// quaternion -> yaw
inline double quatToYaw(const geometry_msgs::msg::Quaternion &q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// Find closest point on path
struct ClosestPointResult {
    size_t idx;
    double distance;
    double x;
    double y;
};

// Lookahead result
struct LookaheadResult {
    double x;
    double y;
    double v;  // interpolated speed
    size_t idx;
    bool found;
};

class PathTrackerNode : public rclcpp::Node {
public:
  PathTrackerNode();

private:
  // Parameters
  double lookahead_base_;
  double lookahead_k_;
  double lookahead_v_;  // velocity-based lookahead gain
  bool use_speed_lookahead_;
  double wheelbase_;
  double default_speed_;
  double max_steering_angle_;
  double path_timeout_;

  // Adaptive lookahead parameters
  bool use_adaptive_lookahead_;
  double lookahead_min_;
  double lookahead_max_;
  double k_curvature_;       // curvature-based lookahead gain
  double k_error_;           // lateral error-based lookahead gain
  double curvature_epsilon_; // small value to avoid division by zero

  // Stanley controller parameters
  bool use_stanley_;
  double stanley_k_;         // Stanley gain for lateral error correction

  // Steering filter parameters
  bool use_steering_filter_;
  double steering_alpha_;    // Low-pass filter coefficient (0-1)
  double prev_steering_;     // Previous steering angle for filtering
  double prev_steering_for_accel_;  // Previous steering for acceleration lookup

  // Debug mode parameters
  bool debug_mode_;
  double velocity_gain_;
  double debug_min_speed_;

  // Speed control modes
  enum class SpeedMode {
    DEFAULT,           // Use default_speed
    PATH_VELOCITY,     // Use velocity from path points
    CURVATURE_BASED,   // Calculate from curvature
    OPTIMIZE           // Use minimum of path_velocity, curvature, and steering_limit
  };
  SpeedMode speed_mode_;

  // Curvature-based speed parameters
  double friction_coeff_;     // friction coefficient (mu)
  double max_speed_limit_;    // maximum speed limit
  double min_speed_limit_;    // minimum speed limit

  // Acceleration limiting parameters (friction circle based)
  bool use_acceleration_limit_;
  double max_total_acceleration_;        // Maximum total acceleration (friction circle limit) [m/s^2]
  double prev_commanded_speed_;          // Previous commanded speed for rate limiting
  std::vector<double> steering_angles_;  // Lookup table steering angles
  std::vector<double> velocities_;       // Lookup table velocities
  std::vector<std::vector<double>> accel_table_;  // 2D lateral acceleration table [m/s^2]

  std::string odom_topic_, drive_topic_, path_topic_, global_path_topic_;
  std::string base_frame_;

  // Path data
  std::vector<PathPoint> current_path_;
  std::vector<PathPoint> global_path_;  // for velocity reference
  rclcpp::Time last_path_time_;
  bool path_received_;
  bool global_path_received_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;

  // Callbacks
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Path tracking functions
  ClosestPointResult findClosestPoint(double px, double py);
  LookaheadResult findLookaheadPoint(size_t start_idx, double lookahead_dist);
  double computeSteeringAngle(double px, double py, double yaw,
                              double goal_x, double goal_y);

  // Adaptive lookahead functions
  double computeCurvatureAtPoint(size_t idx);
  double computeLateralError(double px, double py, const ClosestPointResult& closest);
  double computeAdaptiveLookahead(double base_lookahead, double curvature,
                                   double lateral_error, double velocity);

  // Stanley controller functions
  double computeStanleyTerm(double lateral_error, double velocity, double path_yaw, double vehicle_yaw);
  double applySteeringFilter(double raw_steering);

  // Speed computation functions
  double computeSpeed(const LookaheadResult& lookahead, double current_speed);
  double getGlobalPathSpeed(double x, double y);
  double computeCurvatureSpeed(size_t idx);
  double computeCurvature(const PathPoint& p1, const PathPoint& p2, const PathPoint& p3);

  // Acceleration limiting functions (friction circle based)
  bool loadLateralAccelerationLookupTable(const std::string& filename);
  double getMaxLateralAccelerationFromTable(double steering_angle, double current_speed);
  double computeCurrentLateralAcceleration(double steering_angle, double current_speed);
  double computeMaxLongitudinalAcceleration(double lateral_accel);
  double applyAccelerationLimit(double target_speed, double current_speed, double dt, double steering_angle);

  // Visualization
  void visualizeLookahead(double x, double y);
};

#endif
