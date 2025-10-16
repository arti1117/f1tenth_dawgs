// path_tracker_node.cpp

#include "path_tracker/path_tracker_node.hpp"
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

PathTrackerNode::PathTrackerNode() : Node("path_tracker"), path_received_(false), global_path_received_(false) {
    // Declare parameters
    this->declare_parameter<double>("lookahead_base", 1.5);
    this->declare_parameter<double>("lookahead_k", 0.3);
    this->declare_parameter<double>("wheelbase", 0.33);
    this->declare_parameter<double>("default_speed", 2.0);
    this->declare_parameter<double>("max_steering_angle", 0.4189);  // 24 degrees
    this->declare_parameter<double>("path_timeout", 1.0);
    this->declare_parameter<bool>("use_speed_lookahead", true);

    // Speed mode parameters
    this->declare_parameter<std::string>("speed_mode", "default");  // "default", "path_velocity", "curvature"
    this->declare_parameter<double>("friction_coeff", 0.9);
    this->declare_parameter<double>("max_speed_limit", 8.0);
    this->declare_parameter<double>("min_speed_limit", 0.5);

    // Topics
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("path_topic", "/frenet_path");
    this->declare_parameter<std::string>("global_path_topic", "/global_centerline");
    this->declare_parameter<std::string>("base_frame", "base_link");

    // Get parameters
    lookahead_base_ = this->get_parameter("lookahead_base").as_double();
    lookahead_k_ = this->get_parameter("lookahead_k").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    default_speed_ = this->get_parameter("default_speed").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    path_timeout_ = this->get_parameter("path_timeout").as_double();
    use_speed_lookahead_ = this->get_parameter("use_speed_lookahead").as_bool();

    // Speed mode
    std::string speed_mode_str = this->get_parameter("speed_mode").as_string();
    if (speed_mode_str == "path_velocity") {
        speed_mode_ = SpeedMode::PATH_VELOCITY;
    } else if (speed_mode_str == "curvature") {
        speed_mode_ = SpeedMode::CURVATURE_BASED;
    } else if (speed_mode_str == "optimize") {
        speed_mode_ = SpeedMode::OPTIMIZE;
    } else {
        speed_mode_ = SpeedMode::DEFAULT;
    }

    friction_coeff_ = this->get_parameter("friction_coeff").as_double();
    max_speed_limit_ = this->get_parameter("max_speed_limit").as_double();
    min_speed_limit_ = this->get_parameter("min_speed_limit").as_double();

    // Steering-based speed limiting parameters
    this->declare_parameter<bool>("use_steering_limit", false);
    this->declare_parameter<std::string>("steering_lookup_table", "NUC6_glc_pacejka_lookup_table.csv");
    this->declare_parameter<std::string>("package_share_dir", "/home/dawgs_nx/f1tenth_dawgs/src/controller/path_tracker/config");
    this->declare_parameter<double>("max_lateral_accel", 9.81);

    use_steering_limit_ = this->get_parameter("use_steering_limit").as_bool();
    max_lateral_accel_ = this->get_parameter("max_lateral_accel").as_double();

    // Load steering lookup table if enabled
    if (use_steering_limit_) {
        std::string lookup_table_file = this->get_parameter("steering_lookup_table").as_string();
        std::string package_share_dir = this->get_parameter("package_share_dir").as_string();
        std::string lookup_table_path = package_share_dir + "/" + lookup_table_file;

        if (!loadSteeringLookupTable(lookup_table_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load steering lookup table, disabling steering limit");
            use_steering_limit_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Steering-based speed limiting enabled with table: %s",
                        lookup_table_file.c_str());
        }
    }

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    global_path_topic_ = this->get_parameter("global_path_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&PathTrackerNode::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, 10, std::bind(&PathTrackerNode::pathCallback, this, std::placeholders::_1));

    // Subscribe to global path for velocity reference (if needed)
    if (speed_mode_ == SpeedMode::PATH_VELOCITY || speed_mode_ == SpeedMode::OPTIMIZE) {
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            global_path_topic_, 10, std::bind(&PathTrackerNode::globalPathCallback, this, std::placeholders::_1));
    }

    // Publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    lookahead_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_point", 10);

    RCLCPP_INFO(this->get_logger(), "Path Tracker Node initialized");
    RCLCPP_INFO(this->get_logger(), "Listening to path topic: %s", path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Speed mode: %s", speed_mode_str.c_str());
    if (speed_mode_ == SpeedMode::PATH_VELOCITY || speed_mode_ == SpeedMode::OPTIMIZE) {
        RCLCPP_INFO(this->get_logger(), "Listening to global path topic: %s", global_path_topic_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Lookahead: base=%.2f m, k=%.2f", lookahead_base_, lookahead_k_);
}

void PathTrackerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Received empty path");
        return;
    }

    // Convert path message to internal representation
    current_path_.clear();
    int zero_velocity_count = 0;
    int nonzero_velocity_count = 0;

    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const auto& pose = msg->poses[i];
        PathPoint pt;
        pt.x = pose.pose.position.x;
        pt.y = pose.pose.position.y;
        pt.yaw = quatToYaw(pose.pose.orientation);

        // Read velocity from pose.position.z (stored by path_planner)
        // If z is near zero, use default speed
        pt.v = (pose.pose.position.z > 0.01) ? pose.pose.position.z : default_speed_;

        if (pose.pose.position.z > 0.01) {
            nonzero_velocity_count++;
        } else {
            zero_velocity_count++;
        }

        // Log first 3 points for debugging
        if (i < 3) {
            RCLCPP_INFO(this->get_logger(),
                "PATH_CALLBACK: Point[%zu]: pose.z=%.4f → pt.v=%.2f m/s",
                i, pose.pose.position.z, pt.v);
        }

        current_path_.push_back(pt);
    }

    last_path_time_ = this->now();
    path_received_ = true;

    auto min_v = (*std::min_element(current_path_.begin(), current_path_.end(),
        [](const PathPoint& a, const PathPoint& b) { return a.v < b.v; })).v;
    auto max_v = (*std::max_element(current_path_.begin(), current_path_.end(),
        [](const PathPoint& a, const PathPoint& b) { return a.v < b.v; })).v;

    RCLCPP_INFO(this->get_logger(),
        "PATH_CALLBACK: Received path with %zu points | Velocity stats: min=%.2f, max=%.2f m/s | "
        "Points with velocity: %d, Points using default: %d",
        current_path_.size(), min_v, max_v, nonzero_velocity_count, zero_velocity_count);
}

void PathTrackerNode::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        return;
    }

    // Convert global path (with velocity information from CSV)
    global_path_.clear();
    for (const auto& pose : msg->poses) {
        PathPoint pt;
        pt.x = pose.pose.position.x;
        pt.y = pose.pose.position.y;
        pt.yaw = quatToYaw(pose.pose.orientation);

        // Try to extract velocity from pose.position.z (if path_planner stores it there)
        // Otherwise, use a default value
        pt.v = (pose.pose.position.z > 0.01) ? pose.pose.position.z : default_speed_;

        global_path_.push_back(pt);
    }

    global_path_received_ = true;

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Received global path with %zu points", global_path_.size());
}

void PathTrackerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Check if we have a valid path (must receive frenet_path first)
    if (!path_received_ || current_path_.size() < 2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "No valid frenet_path available for tracking (waiting for path_planner)");

        // Publish zero command to stop vehicle
        ackermann_msgs::msg::AckermannDriveStamped stop_msg;
        stop_msg.header.stamp = this->now();
        stop_msg.header.frame_id = base_frame_;
        stop_msg.drive.steering_angle = 0.0;
        stop_msg.drive.speed = 0.0;
        drive_pub_->publish(stop_msg);
        return;
    }

    // Check path timeout
    double path_age = (this->now() - last_path_time_).seconds();
    if (path_age > path_timeout_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Path is stale (%.2f s old), waiting for new path", path_age);

        // Publish zero command to stop vehicle
        ackermann_msgs::msg::AckermannDriveStamped stop_msg;
        stop_msg.header.stamp = this->now();
        stop_msg.header.frame_id = base_frame_;
        stop_msg.drive.steering_angle = 0.0;
        stop_msg.drive.speed = 0.0;
        drive_pub_->publish(stop_msg);
        return;
    }

    // Get current pose
    double px = msg->pose.pose.position.x;
    double py = msg->pose.pose.position.y;
    double yaw = quatToYaw(msg->pose.pose.orientation);
    double vx = msg->twist.twist.linear.x;
    double v = std::hypot(vx, msg->twist.twist.linear.y);

    // Find closest point on path
    auto closest = findClosestPoint(px, py);

    if (closest.distance > 5.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Vehicle too far from path: %.2f m", closest.distance);
        return;
    }

    // Compute lookahead distance
    double lookahead_dist = lookahead_base_;
    if (use_speed_lookahead_) {
        lookahead_dist = lookahead_base_ + lookahead_k_ * std::max(0.0, v);
    }
    lookahead_dist = std::max(0.5, lookahead_dist);  // Minimum lookahead

    // Find lookahead point
    auto lookahead = findLookaheadPoint(closest.idx, lookahead_dist);

    if (!lookahead.found) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Could not find lookahead point");
        return;
    }

    // Debug log: lookahead point velocity
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "LOOKAHEAD: idx=%zu, lookahead.v=%.4f m/s (from interpolation at lookahead point)",
        lookahead.idx, lookahead.v);

    // Visualize lookahead point
    visualizeLookahead(lookahead.x, lookahead.y);

    // Compute steering angle using pure pursuit
    double steering = computeSteeringAngle(px, py, yaw, lookahead.x, lookahead.y);

    // Clamp steering angle
    steering = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering));

    // Compute speed based on selected mode
    double target_speed = computeSpeed(lookahead, v);

    // Apply steering-based speed limiting (if enabled)
    if (use_steering_limit_) {
        double max_safe_speed = computeMaxSafeSpeed(steering, v);
        double speed_before_limit = target_speed;
        target_speed = std::min(target_speed, max_safe_speed);

        if (speed_before_limit > target_speed + 0.1) {  // Log only significant reductions
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Speed limited by steering: %.2f → %.2f m/s (steering=%.3f rad)",
                speed_before_limit, target_speed, steering);
        }
    }

    // Publish drive command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = base_frame_;
    drive_msg.drive.steering_angle = steering;
    drive_msg.drive.speed = target_speed;
    drive_pub_->publish(drive_msg);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Tracking: dist_to_path=%.2f m, lookahead=%.2f m, steering=%.3f rad, speed=%.2f m/s",
        closest.distance, lookahead_dist, steering, drive_msg.drive.speed);
}

ClosestPointResult PathTrackerNode::findClosestPoint(double px, double py) {
    ClosestPointResult result;
    result.idx = 0;
    result.distance = std::numeric_limits<double>::infinity();
    result.x = 0.0;
    result.y = 0.0;

    for (size_t i = 0; i < current_path_.size(); ++i) {
        double dx = current_path_[i].x - px;
        double dy = current_path_[i].y - py;
        double dist = std::hypot(dx, dy);

        if (dist < result.distance) {
            result.distance = dist;
            result.idx = i;
            result.x = current_path_[i].x;
            result.y = current_path_[i].y;
        }
    }

    return result;
}

LookaheadResult PathTrackerNode::findLookaheadPoint(size_t start_idx, double lookahead_dist) {
    LookaheadResult result;
    result.found = false;
    result.idx = start_idx;
    result.x = 0.0;
    result.y = 0.0;
    result.v = default_speed_;

    if (current_path_.empty()) return result;

    // Get starting position
    double px = current_path_[start_idx].x;
    double py = current_path_[start_idx].y;

    // Walk along path accumulating distance
    double accumulated_dist = 0.0;
    size_t idx = start_idx;

    while (idx < current_path_.size() - 1) {
        double dx = current_path_[idx + 1].x - current_path_[idx].x;
        double dy = current_path_[idx + 1].y - current_path_[idx].y;
        double segment_length = std::hypot(dx, dy);

        if (accumulated_dist + segment_length >= lookahead_dist) {
            // Lookahead point is on this segment
            double remaining = lookahead_dist - accumulated_dist;
            double t = (segment_length > 1e-6) ? (remaining / segment_length) : 0.0;
            t = std::max(0.0, std::min(1.0, t));

            result.x = current_path_[idx].x + t * dx;
            result.y = current_path_[idx].y + t * dy;
            result.v = current_path_[idx].v + t * (current_path_[idx + 1].v - current_path_[idx].v);
            result.idx = idx + 1;
            result.found = true;
            return result;
        }

        accumulated_dist += segment_length;
        idx++;
    }

    // If we reach here, use the last point
    result.x = current_path_.back().x;
    result.y = current_path_.back().y;
    result.v = current_path_.back().v;
    result.idx = current_path_.size() - 1;
    result.found = true;

    return result;
}

double PathTrackerNode::computeSteeringAngle(double px, double py, double yaw,
                                              double goal_x, double goal_y) {
    // Transform goal to vehicle frame
    double dx = goal_x - px;
    double dy = goal_y - py;

    // Rotate to vehicle frame
    double x_veh =  std::cos(-yaw) * dx - std::sin(-yaw) * dy;
    double y_veh =  std::sin(-yaw) * dx + std::cos(-yaw) * dy;

    // Pure pursuit formula
    double L = std::hypot(x_veh, y_veh);  // Distance to lookahead point
    if (L < 1e-6) L = 1e-6;  // Avoid division by zero

    double alpha = std::atan2(y_veh, x_veh);
    double steering = std::atan2(2.0 * wheelbase_ * std::sin(alpha), L);

    return steering;
}

void PathTrackerNode::visualizeLookahead(double x, double y) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    lookahead_pub_->publish(marker);
}

double PathTrackerNode::computeSpeed(const LookaheadResult& lookahead, double current_speed) {
    double speed = default_speed_;

    switch (speed_mode_) {
        case SpeedMode::PATH_VELOCITY: {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "COMPUTE_SPEED [PATH_VELOCITY]: lookahead.v=%.4f, threshold=0.01",
                lookahead.v);

            // First try to use velocity from the current path (frenet_path with embedded velocity)
            if (lookahead.v > 0.01) {
                speed = lookahead.v;
                double clamped_speed = std::max(min_speed_limit_, std::min(max_speed_limit_, speed));
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "PATH_VELOCITY: Using frenet path velocity | raw=%.4f → clamped=%.2f m/s (limits: [%.2f, %.2f])",
                    lookahead.v, clamped_speed, min_speed_limit_, max_speed_limit_);
            } else {
                // Fallback to global path velocity if frenet path doesn't have velocity
                speed = getGlobalPathSpeed(lookahead.x, lookahead.y);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "PATH_VELOCITY: Fallback to global path | lookahead.v=%.4f < 0.01 → using global_speed=%.2f m/s",
                    lookahead.v, speed);
            }
            break;
        }

        case SpeedMode::CURVATURE_BASED:
            // Calculate speed based on path curvature
            speed = computeCurvatureSpeed(lookahead.idx);
            break;

        case SpeedMode::OPTIMIZE: {
            // Calculate all three speed limits
            double path_speed;
            if (lookahead.v > 0.01) {
                path_speed = lookahead.v;
            } else {
                path_speed = getGlobalPathSpeed(lookahead.x, lookahead.y);
            }

            double curv_speed = computeCurvatureSpeed(lookahead.idx);

            // Note: steering limit is applied separately in odomCallback after computeSpeed
            // Here we just use the minimum of path and curvature speeds
            speed = std::min(path_speed, curv_speed);

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "OPTIMIZE: path=%.2f, curv=%.2f → using %.2f m/s (steering limit applied separately)",
                path_speed, curv_speed, speed);
            break;
        }

        case SpeedMode::DEFAULT:
        default:
            // Use default speed
            speed = default_speed_;
            break;
    }

    // Clamp speed to limits
    speed = std::max(min_speed_limit_, std::min(max_speed_limit_, speed));

    return speed;
}

double PathTrackerNode::getGlobalPathSpeed(double x, double y) {
    if (!global_path_received_ || global_path_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Global path not available, using default speed");
        return default_speed_;
    }

    // Find closest point on global path to lookahead point
    double min_dist = std::numeric_limits<double>::infinity();
    size_t closest_idx = 0;

    for (size_t i = 0; i < global_path_.size(); ++i) {
        double dx = global_path_[i].x - x;
        double dy = global_path_[i].y - y;
        double dist = std::hypot(dx, dy);

        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // Return velocity at closest point
    return global_path_[closest_idx].v;
}

double PathTrackerNode::computeCurvatureSpeed(size_t idx) {
    if (current_path_.size() < 3) {
        return default_speed_;
    }

    // Get three points for curvature calculation
    size_t prev_idx = (idx > 0) ? idx - 1 : 0;
    size_t curr_idx = std::min(idx, current_path_.size() - 1);
    size_t next_idx = std::min(idx + 1, current_path_.size() - 1);

    // Compute curvature
    double kappa = computeCurvature(
        current_path_[prev_idx],
        current_path_[curr_idx],
        current_path_[next_idx]
    );

    // Compute maximum safe speed based on curvature
    // v_max = sqrt(mu * g / |kappa|)
    const double g = 9.81;  // gravity
    double kappa_abs = std::abs(kappa);

    if (kappa_abs < 1e-6) {
        // Essentially straight, use max speed
        return max_speed_limit_;
    }

    double v_max = std::sqrt(friction_coeff_ * g / kappa_abs);

    return v_max;
}

double PathTrackerNode::computeCurvature(const PathPoint& p1, const PathPoint& p2, const PathPoint& p3) {
    // Compute curvature using three points
    // Formula: kappa = 2 * |cross(v1, v2)| / |v1|^3
    // where v1 = p2 - p1, v2 = p3 - p2

    double v1x = p2.x - p1.x;
    double v1y = p2.y - p1.y;
    double v2x = p3.x - p2.x;
    double v2y = p3.y - p2.y;

    // Cross product in 2D (z-component)
    double cross = v1x * v2y - v1y * v2x;

    // Magnitude of v1
    double v1_mag = std::hypot(v1x, v1y);

    if (v1_mag < 1e-6) {
        return 0.0;  // Avoid division by zero
    }

    // Curvature
    double kappa = 2.0 * cross / (v1_mag * v1_mag * v1_mag);

    return kappa;
}

bool PathTrackerNode::loadSteeringLookupTable(const std::string& filename) {
    // Open CSV file
    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open steering lookup table: %s", filename.c_str());
        return false;
    }

    std::string line;
    std::vector<std::vector<double>> data;

    // Read CSV line by line
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            try {
                row.push_back(std::stod(cell));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse CSV cell: %s", cell.c_str());
                return false;
            }
        }

        if (!row.empty()) {
            data.push_back(row);
        }
    }

    file.close();

    // Parse table structure
    // First row: velocities (skip first element which is 0)
    // First column: steering angles (skip first element)
    // Rest: acceleration values

    if (data.size() < 2 || data[0].size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Invalid lookup table dimensions");
        return false;
    }

    // Extract velocities from first row (skip index 0)
    velocities_.clear();
    for (size_t i = 1; i < data[0].size(); ++i) {
        velocities_.push_back(data[0][i]);
    }

    // Extract steering angles and acceleration table
    steering_angles_.clear();
    accel_table_.clear();

    for (size_t i = 1; i < data.size(); ++i) {
        // First element is steering angle
        steering_angles_.push_back(data[i][0]);

        // Rest are acceleration values
        std::vector<double> accel_row;
        for (size_t j = 1; j < data[i].size(); ++j) {
            accel_row.push_back(data[i][j]);
        }
        accel_table_.push_back(accel_row);
    }

    RCLCPP_INFO(this->get_logger(),
                "Loaded steering lookup table: %zu steering angles × %zu velocities",
                steering_angles_.size(), velocities_.size());

    return true;
}

double PathTrackerNode::computeMaxSafeSpeed(double steering_angle, double current_speed) {
    if (!use_steering_limit_ || steering_angles_.empty() || velocities_.empty()) {
        return max_speed_limit_;  // No limit if not enabled
    }

    // Use absolute steering angle
    double abs_steering = std::abs(steering_angle);

    // Find the two closest steering angles in the lookup table
    size_t lower_idx = 0;
    size_t upper_idx = 0;

    for (size_t i = 0; i < steering_angles_.size(); ++i) {
        if (steering_angles_[i] <= abs_steering) {
            lower_idx = i;
        } else {
            upper_idx = i;
            break;
        }
    }

    // If beyond table range, use last index
    if (upper_idx == 0 && abs_steering > steering_angles_.back()) {
        lower_idx = steering_angles_.size() - 1;
        upper_idx = steering_angles_.size() - 1;
    }

    // For each velocity in the table, check if the vehicle can maintain it
    // at the given steering angle with acceptable lateral acceleration
    double max_safe_vel = max_speed_limit_;

    for (int v_idx = velocities_.size() - 1; v_idx >= 0; --v_idx) {
        double velocity = velocities_[v_idx];

        // Interpolate acceleration at this steering angle and velocity
        double accel_lower = (lower_idx < accel_table_.size() && v_idx < accel_table_[lower_idx].size())
                             ? accel_table_[lower_idx][v_idx] : 0.0;
        double accel_upper = (upper_idx < accel_table_.size() && v_idx < accel_table_[upper_idx].size())
                             ? accel_table_[upper_idx][v_idx] : 0.0;

        double accel;
        if (lower_idx == upper_idx) {
            accel = accel_lower;
        } else {
            // Linear interpolation between steering angles
            double t = (abs_steering - steering_angles_[lower_idx]) /
                       (steering_angles_[upper_idx] - steering_angles_[lower_idx]);
            accel = accel_lower + t * (accel_upper - accel_lower);
        }

        // Check if lateral acceleration is within safe limits
        // The lookup table stores maximum achievable lateral acceleration
        // We want velocity where required lateral accel <= table value

        // Required lateral acceleration: a_lat = v^2 / R = v^2 * kappa
        // For small angles: kappa ≈ steering_angle / wheelbase
        double curvature = abs_steering / wheelbase_;
        double required_lateral_accel = velocity * velocity * curvature;

        // If the achievable acceleration at this velocity is sufficient
        // and within our max limit, this velocity is safe
        if (required_lateral_accel <= max_lateral_accel_ &&
            required_lateral_accel <= accel * 10.0) {  // accel in table is normalized
            max_safe_vel = velocity;
            break;
        }
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Steering limit: angle=%.3f rad, max_safe_speed=%.2f m/s",
                          steering_angle, max_safe_vel);

    return max_safe_vel;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
