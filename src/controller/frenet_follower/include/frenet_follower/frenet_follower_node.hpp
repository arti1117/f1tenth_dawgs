#pragma once

#ifndef FRENET_FOLLOWER_HPP
#define FRENET_FOLLOWER_HPP

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"

struct PathPoint {
  double x;
  double y;
  double yaw;
  double v;        // desired speed at this point
  double kappa;    // curvature at this point
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
    double yaw;
};

class FrenetFollowerNode : public rclcpp::Node {
public:
  FrenetFollowerNode();

private:
  // =============================================================================
  // STEERING CONTROL PARAMETERS
  // =============================================================================
  // The frenet_follower uses a direct path following approach that combines:
  // 1. Heading error control (align vehicle heading with path heading)
  // 2. Lateral error feedback (reduce cross-track distance)
  // 3. Curvature feedforward (anticipate path curvature)

  // Heading control
  double k_heading_;           // Heading error gain (higher = more aggressive alignment)

  // Lateral error control (Stanley-inspired)
  double k_lateral_;           // Lateral error gain for cross-track correction
  double k_lateral_velocity_;  // Velocity-dependent lateral gain reduction

  // Curvature feedforward
  bool use_curvature_feedforward_;    // Enable curvature-based steering
  double k_curvature_;                // Curvature feedforward gain
  double curvature_epsilon_;          // Small value to avoid division by zero

  // Steering filter
  bool use_steering_filter_;
  double steering_alpha_;      // Low-pass filter coefficient (0-1)
  double prev_steering_;       // Previous steering angle for filtering

  // Vehicle parameters
  double wheelbase_;
  double max_steering_angle_;

  // Path tracking parameters
  double path_timeout_;
  double max_lateral_error_;   // Maximum allowed lateral error before stopping

  // Forward tracking parameters (for low odom frequency)
  bool use_forward_tracking_;       // Enable sequential path following
  double forward_search_range_;     // Forward search range [m] from last target
  size_t last_target_idx_;          // Last target index on path

  // =============================================================================
  // SPEED CONTROL PARAMETERS (from path_tracker)
  // =============================================================================
  double default_speed_;
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
  double friction_coeff_;
  double max_speed_limit_;
  double min_speed_limit_;

  // Debug mode
  bool debug_mode_;
  double velocity_gain_;

  // Simulation mode
  bool sim_mode_;

  // Position compensation
  bool use_position_compensation_;
  double expected_computation_time_;

  // Acceleration limiting (friction circle)
  bool use_acceleration_limit_;
  double max_total_acceleration_;
  double prev_commanded_speed_;
  std::vector<double> steering_angles_;
  std::vector<double> velocities_;
  std::vector<std::vector<double>> accel_table_;

  // Topics
  std::string odom_topic_, drive_topic_, path_topic_, global_path_topic_;
  std::string base_frame_;

  // Path data
  std::vector<PathPoint> current_path_;
  std::vector<PathPoint> global_path_;
  rclcpp::Time last_path_time_;
  bool path_received_;
  bool global_path_received_;

  // Drive command buffer
  ackermann_msgs::msg::AckermannDriveStamped last_drive_cmd_;
  bool has_drive_cmd_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_pub_;

  // Callbacks
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Path tracking functions
  ClosestPointResult findClosestPoint(double px, double py);

  // Optimized steering calculation (direct path following)
  double computeOptimizedSteering(double px, double py, double vehicle_yaw,
                                   const ClosestPointResult& closest,
                                   double velocity);

  double computeHeadingError(double vehicle_yaw, double path_yaw);
  double computeLateralError(double px, double py, const ClosestPointResult& closest);
  double computeCurvatureAtPoint(size_t idx);
  double computeCurvature(const PathPoint& p1, const PathPoint& p2, const PathPoint& p3);
  double applySteeringFilter(double raw_steering);

  // Speed computation functions (from path_tracker)
  double computeSpeed(const PathPoint& target_point, double current_speed);
  double getGlobalPathSpeed(double x, double y);
  double computeCurvatureSpeed(size_t idx);

  // Acceleration limiting functions
  bool loadLateralAccelerationLookupTable(const std::string& filename);
  double getMaxLateralAccelerationFromTable(double steering_angle, double current_speed);
  double computeCurrentLateralAcceleration(double steering_angle, double current_speed);
  double computeMaxLongitudinalAcceleration(double lateral_accel);
  double applyAccelerationLimit(double target_speed, double current_speed, double dt, double steering_angle);

  // Path interpolation
  void interpolatePath(const std::vector<PathPoint>& coarse_path,
                       std::vector<PathPoint>& fine_path,
                       double resolution = 0.1);

  // Visualization
  void visualizeTargetPoint(double x, double y);
};

#endif
