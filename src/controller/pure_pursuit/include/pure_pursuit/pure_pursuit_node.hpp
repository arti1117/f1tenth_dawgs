#pragma once

#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nanoflann.hpp" // make sure include path points to nanoflann.hpp

struct Waypoint {
  double x;
  double y;
  double v; // desired speed at waypoint
};

// KD-tree wrapper for nanoflann
struct WaypointCloud {
  std::vector<Waypoint> pts;
  inline size_t kdtree_get_point_count() const { return pts.size(); }
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0) return pts[idx].x;
    return pts[idx].y;
  }
  template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};
using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
  nanoflann::L2_Simple_Adaptor<double, WaypointCloud>,
  WaypointCloud, 2>;

// quaternion -> yaw
inline double quatToYaw(const geometry_msgs::msg::Quaternion &q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// ---------- projection (local window) ----------
struct Projection {
    double s;
    double lateral;
    int seg_idx;
    bool valid;
};

// ---------- find lookahead point with exact segment interpolation ----------
// start_idx = index of nearest waypoint
// returns interpolated (x,y) and interpolated v and segment index
struct LookaheadResult {
    double x;
    double y;
    double v;
    int idx; // index of point after interpolation (the 'next' waypoint index)
    double seg_t; // interpolation t on that segment
};

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode();

  ~PurePursuitNode() {
    if (kdtree_) delete kdtree_;
    if (csv_out_.is_open()) csv_out_.close();
  }

private:
  // params
  std::string waypoint_file_;
  double lookahead_base_;
  double lookahead_k_;
  bool use_speed_lookahead_;
  double wheelbase_;
  double lateral_threshold_;
  double v_min_lap_;
  double min_lap_interval_;
  int local_window_;
  std::string csv_log_path_;
  bool rotate_on_start_;

  std::string odom_topic_, drive_topic_, lap_topic_;
  std::string wp_topic_, str_topic_, wp_near_topic_, wp_ahead_topic_;
  std::string base_frame_;

  // data
  WaypointCloud cloud_;
  KDTree* kdtree_ = nullptr;
  std::vector<double> s_lookup_;
  double S_total_ = 0.0;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lap_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr str_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wp_near_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wp_ahead_pub_;


  // lap state
  double s_prev_ = 0.0;
  bool s_prev_initialized_;
  rclcpp::Time last_lap_time_;
  rclcpp::Time lap_start_time_;

  // prev odom for interpolation
  rclcpp::Time prev_odom_time_;
  double prev_s_;
  double prev_px_, prev_py_;

  // logging
  std::ofstream csv_out_;

  // start-rotate flag
  bool rotated_already_;

  bool loadWaypointsCSV(const std::string &fname);

  void buildKDTree();
  void computeSlookup();
  void rotatePathToNearest(double px, double py);
  Projection projectToPathLocal(double px, double py, int nearest_idx);
  LookaheadResult findLookaheadPointInterpolated(int start_idx, double lookahead_dist, int direction);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  double curvature(const Waypoint& p1, const Waypoint& p2, const Waypoint& p3);
  int determineDirection(size_t idx, double yaw);
  double adjustSpeed(const std::vector<Waypoint> & wps, size_t idx, LookaheadResult & la);


  void visualize_waypoint(const std::vector<Waypoint>& wps);
  void visualize_steering(const float speed, const float steering);
  void visualize_nearest_wp(const std::vector<Waypoint>& wps, const int nearest_idx);
  void visualize_lookahead_wp(const std::vector<Waypoint>& wps, const double & x, const double & y);

};
#endif