// frenet_follower_node.cpp
// Direct Frenet path follower with optimized steering control

#include "frenet_follower/frenet_follower_node.hpp"

using namespace std::chrono_literals;

FrenetFollowerNode::FrenetFollowerNode()
    : Node("frenet_follower"),
      path_received_(false),
      global_path_received_(false),
      has_drive_cmd_(false) {

    // =============================================================================
    // STEERING CONTROL PARAMETERS
    // =============================================================================
    this->declare_parameter<double>("k_heading", 1.5);
    this->declare_parameter<double>("k_lateral", 0.8);
    this->declare_parameter<double>("k_lateral_velocity", 0.1);
    this->declare_parameter<bool>("use_curvature_feedforward", true);
    this->declare_parameter<double>("k_curvature", 1.0);
    this->declare_parameter<double>("curvature_epsilon", 0.001);

    // Steering filter
    this->declare_parameter<bool>("use_steering_filter", true);
    this->declare_parameter<double>("steering_alpha", 0.3);

    // Vehicle parameters
    this->declare_parameter<double>("wheelbase", 0.33);
    this->declare_parameter<double>("max_steering_angle", 0.4189);

    // Path tracking
    this->declare_parameter<double>("path_timeout", 1.0);
    this->declare_parameter<double>("max_lateral_error", 3.0);

    // Forward tracking (for low odom frequency)
    this->declare_parameter<bool>("use_forward_tracking", false);
    this->declare_parameter<double>("forward_search_range", 2.0);

    // =============================================================================
    // SPEED CONTROL PARAMETERS
    // =============================================================================
    this->declare_parameter<double>("default_speed", 2.0);
    this->declare_parameter<double>("debug_min_speed", 0.5);
    this->declare_parameter<std::string>("speed_mode", "default");
    this->declare_parameter<double>("friction_coeff", 0.9);
    this->declare_parameter<double>("max_speed_limit", 8.0);
    this->declare_parameter<double>("min_speed_limit", 0.5);

    // Debug and simulation
    this->declare_parameter<bool>("debug_mode", false);
    this->declare_parameter<double>("velocity_gain", 1.0);
    this->declare_parameter<bool>("sim_mode", false);
    this->declare_parameter<std::string>("sim_odom", "/ego_racecar/odom");

    // Position compensation
    this->declare_parameter<bool>("use_position_compensation", true);
    this->declare_parameter<double>("expected_computation_time", 0.01);

    // Acceleration limiting
    this->declare_parameter<bool>("use_acceleration_limit", false);
    this->declare_parameter<std::string>("lateral_accel_lookup_table", "dawgs_lookup_table.csv");
    this->declare_parameter<std::string>("package_share_dir", "/home/dawgs_nx/f1tenth_dawgs/src/controller/frenet_follower/config");
    this->declare_parameter<double>("max_total_acceleration", 9.81);

    // Topics
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("path_topic", "/planned_path");
    this->declare_parameter<std::string>("global_path_topic", "/global_centerline");
    this->declare_parameter<std::string>("base_frame", "base_link");

    // =============================================================================
    // GET PARAMETERS
    // =============================================================================
    // Steering control
    k_heading_ = this->get_parameter("k_heading").as_double();
    k_lateral_ = this->get_parameter("k_lateral").as_double();
    k_lateral_velocity_ = this->get_parameter("k_lateral_velocity").as_double();
    use_curvature_feedforward_ = this->get_parameter("use_curvature_feedforward").as_bool();
    k_curvature_ = this->get_parameter("k_curvature").as_double();
    curvature_epsilon_ = this->get_parameter("curvature_epsilon").as_double();

    // Steering filter
    use_steering_filter_ = this->get_parameter("use_steering_filter").as_bool();
    steering_alpha_ = this->get_parameter("steering_alpha").as_double();
    prev_steering_ = 0.0;

    // Vehicle
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

    // Path tracking
    path_timeout_ = this->get_parameter("path_timeout").as_double();
    max_lateral_error_ = this->get_parameter("max_lateral_error").as_double();

    // Forward tracking
    use_forward_tracking_ = this->get_parameter("use_forward_tracking").as_bool();
    forward_search_range_ = this->get_parameter("forward_search_range").as_double();
    last_target_idx_ = 0;  // Initialize to 0

    // Speed control
    default_speed_ = this->get_parameter("default_speed").as_double();
    debug_min_speed_ = this->get_parameter("debug_min_speed").as_double();

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

    // Debug and simulation
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    velocity_gain_ = this->get_parameter("velocity_gain").as_double();
    sim_mode_ = this->get_parameter("sim_mode").as_bool();

    // Position compensation
    use_position_compensation_ = this->get_parameter("use_position_compensation").as_bool();
    expected_computation_time_ = this->get_parameter("expected_computation_time").as_double();

    // Acceleration limiting
    use_acceleration_limit_ = this->get_parameter("use_acceleration_limit").as_bool();
    max_total_acceleration_ = this->get_parameter("max_total_acceleration").as_double();
    prev_commanded_speed_ = 0.0;

    if (use_acceleration_limit_) {
        std::string lookup_table_file = this->get_parameter("lateral_accel_lookup_table").as_string();
        std::string package_share_dir = this->get_parameter("package_share_dir").as_string();
        std::string lookup_table_path = package_share_dir + "/" + lookup_table_file;

        if (!loadLateralAccelerationLookupTable(lookup_table_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load lateral acceleration lookup table");
            use_acceleration_limit_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Friction circle acceleration limiting enabled");
        }
    }

    // Topics
    odom_topic_ = this->get_parameter("odom_topic").as_string();

    if (sim_mode_) {
        odom_topic_ = this->get_parameter("sim_odom").as_string();
        RCLCPP_INFO(this->get_logger(), "Simulation mode enabled, using: %s", odom_topic_.c_str());
    }

    drive_topic_ = this->get_parameter("drive_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    global_path_topic_ = this->get_parameter("global_path_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    // =============================================================================
    // QoS SETTINGS
    // =============================================================================
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto control_qos = rclcpp::QoS(rclcpp::KeepLast(15)).best_effort();
    auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // =============================================================================
    // SUBSCRIBERS AND PUBLISHERS
    // =============================================================================
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos,
        std::bind(&FrenetFollowerNode::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, path_qos,
        std::bind(&FrenetFollowerNode::pathCallback, this, std::placeholders::_1));

    if (speed_mode_ == SpeedMode::PATH_VELOCITY || speed_mode_ == SpeedMode::OPTIMIZE) {
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            global_path_topic_, path_qos,
            std::bind(&FrenetFollowerNode::globalPathCallback, this, std::placeholders::_1));
    }

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, control_qos);

    target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "frenet_target_point", viz_qos);

    RCLCPP_INFO(this->get_logger(), "=================================================");
    RCLCPP_INFO(this->get_logger(), "Frenet Follower Node Initialized");
    RCLCPP_INFO(this->get_logger(), "=================================================");
    RCLCPP_INFO(this->get_logger(), "Steering Control:");
    RCLCPP_INFO(this->get_logger(), "  - k_heading: %.2f", k_heading_);
    RCLCPP_INFO(this->get_logger(), "  - k_lateral: %.2f", k_lateral_);
    RCLCPP_INFO(this->get_logger(), "  - k_curvature: %.2f (feedforward: %s)",
                k_curvature_, use_curvature_feedforward_ ? "ON" : "OFF");
    RCLCPP_INFO(this->get_logger(), "Path Tracking:");
    RCLCPP_INFO(this->get_logger(), "  - Forward tracking: %s (range: %.2fm)",
                use_forward_tracking_ ? "ENABLED" : "disabled", forward_search_range_);
    RCLCPP_INFO(this->get_logger(), "Speed Control: %s", speed_mode_str.c_str());
    RCLCPP_INFO(this->get_logger(), "Topics:");
    RCLCPP_INFO(this->get_logger(), "  - Odometry: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Path: %s", path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Drive: %s", drive_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "=================================================");
}

void FrenetFollowerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Received empty path");
        return;
    }

    // Convert path message to internal representation
    std::vector<PathPoint> coarse_path;

    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const auto& pose = msg->poses[i];
        PathPoint pt;
        pt.x = pose.pose.position.x;
        pt.y = pose.pose.position.y;
        pt.yaw = quatToYaw(pose.pose.orientation);
        pt.v = (pose.pose.position.z > 0.01) ? pose.pose.position.z : default_speed_;
        pt.kappa = 0.0;  // Will be computed later

        coarse_path.push_back(pt);
    }

    // Interpolate for finer resolution
    current_path_.clear();
    interpolatePath(coarse_path, current_path_, 0.1);

    // Compute curvature for each point
    for (size_t i = 0; i < current_path_.size(); ++i) {
        current_path_[i].kappa = computeCurvatureAtPoint(i);
    }

    last_path_time_ = this->now();
    path_received_ = true;

    // Reset target index when new path is received
    last_target_idx_ = 0;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Received path: %zu points → %zu interpolated points",
        coarse_path.size(), current_path_.size());
}

void FrenetFollowerNode::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        return;
    }

    global_path_.clear();
    for (const auto& pose : msg->poses) {
        PathPoint pt;
        pt.x = pose.pose.position.x;
        pt.y = pose.pose.position.y;
        pt.yaw = quatToYaw(pose.pose.orientation);
        pt.v = (pose.pose.position.z > 0.01) ? pose.pose.position.z : default_speed_;
        pt.kappa = 0.0;

        global_path_.push_back(pt);
    }

    global_path_received_ = true;
}

void FrenetFollowerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Check if we have a valid path
    if (!path_received_ || current_path_.size() < 2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "No valid path available for tracking");

        if (has_drive_cmd_) {
            last_drive_cmd_.header.stamp = this->now();
            drive_pub_->publish(last_drive_cmd_);
        } else {
            ackermann_msgs::msg::AckermannDriveStamped stop_msg;
            stop_msg.header.stamp = this->now();
            stop_msg.header.frame_id = base_frame_;
            stop_msg.drive.steering_angle = 0.0;
            stop_msg.drive.speed = 0.0;
            drive_pub_->publish(stop_msg);
        }
        return;
    }

    // Check path timeout
    double path_age = (this->now() - last_path_time_).seconds();
    if (path_age > path_timeout_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Path is stale (%.2f s old)", path_age);

        if (has_drive_cmd_) {
            last_drive_cmd_.header.stamp = this->now();
            drive_pub_->publish(last_drive_cmd_);
        }
        return;
    }

    // Get current pose and velocity
    double px_raw = msg->pose.pose.position.x;
    double py_raw = msg->pose.pose.position.y;
    double yaw = quatToYaw(msg->pose.pose.orientation);
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double v = std::hypot(vx, vy);

    // Apply position compensation
    double px = px_raw;
    double py = py_raw;

    if (use_position_compensation_ && v > 0.1) {
        auto current_ros_time = this->now();
        auto odom_stamp = rclcpp::Time(msg->header.stamp);
        double message_delay = (current_ros_time - odom_stamp).seconds();
        double total_lookahead_time = message_delay + expected_computation_time_;

        px = px_raw + v * std::cos(yaw) * total_lookahead_time;
        py = py_raw + v * std::sin(yaw) * total_lookahead_time;
    }

    // =============================================================================
    // OPTIMIZED STEERING CALCULATION (Direct Path Following)
    // =============================================================================
    // This controller directly follows the planned path without lookahead points.
    // It combines three control components for optimal path tracking:
    //
    // 1. HEADING ERROR CONTROL: Aligns vehicle heading with path heading
    // 2. LATERAL ERROR FEEDBACK: Reduces cross-track distance (Stanley-inspired)
    // 3. CURVATURE FEEDFORWARD: Anticipates path curvature for proactive steering
    //
    // Benefits over Pure Pursuit:
    // - No lookahead distance tuning required
    // - More precise path following at all speeds
    // - Better corner entry and exit behavior
    // - Reduced computational complexity
    // =============================================================================

    // STEP 1: Find closest point on path
    // -----------------------------------------------------------------------------
    // This serves as our reference point for all error calculations
    auto closest = findClosestPoint(px, py);

    if (closest.distance > max_lateral_error_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Vehicle too far from path: %.2f m (max: %.2f m)",
            closest.distance, max_lateral_error_);
        return;
    }

    // Visualize target point (closest point on path)
    visualizeTargetPoint(closest.x, closest.y);

    // STEP 2: Compute optimized steering angle
    // -----------------------------------------------------------------------------
    // Combines heading error + lateral error + curvature feedforward
    double steering = computeOptimizedSteering(px, py, yaw, closest, v);

    // Apply steering filter for smoothness
    steering = applySteeringFilter(steering);

    // Clamp to physical limits
    steering = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering));

    // STEP 3: Compute target speed
    // -----------------------------------------------------------------------------
    // Uses the same speed calculation as path_tracker
    double target_speed = computeSpeed(current_path_[closest.idx], v);

    // Apply acceleration limiting if enabled
    double commanded_speed = target_speed;
    if (use_acceleration_limit_) {
        double dt = 0.05;  // 20 Hz control loop
        commanded_speed = applyAccelerationLimit(target_speed, v, dt, steering);
    }

    // Publish drive command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = base_frame_;
    drive_msg.drive.steering_angle = steering;
    drive_msg.drive.speed = commanded_speed;

    drive_pub_->publish(drive_msg);

    // Store for next cycle
    last_drive_cmd_ = drive_msg;
    has_drive_cmd_ = true;
    prev_commanded_speed_ = commanded_speed;

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Tracking: dist=%.2f m, steering=%.3f rad, speed=%.2f m/s",
        closest.distance, steering, commanded_speed);
}

ClosestPointResult FrenetFollowerNode::findClosestPoint(double px, double py) {
    ClosestPointResult result;
    result.idx = 0;
    result.distance = std::numeric_limits<double>::infinity();
    result.x = 0.0;
    result.y = 0.0;
    result.yaw = 0.0;

    // =============================================================================
    // FORWARD TRACKING MODE (for low odom frequency)
    // =============================================================================
    // When odom frequency is low, searching the entire path may result in
    // repeatedly targeting the same point. Forward tracking ensures sequential
    // progression along the path by searching only ahead of the last target.
    //
    // Strategy:
    // 1. Calculate search window based on forward_search_range_
    // 2. Convert range to path arc length (approximate)
    // 3. Search only forward indices from last_target_idx_
    // 4. If no point found in forward window, fall back to full search
    // 5. Update last_target_idx_ for next cycle
    // =============================================================================

    if (use_forward_tracking_ && current_path_.size() > 0) {
        // Calculate forward search window size (arc length approximation)
        // Assume average spacing ~0.1m (interpolation resolution)
        size_t forward_window = static_cast<size_t>(forward_search_range_ / 0.1);

        // Define search range: [start_idx, end_idx)
        size_t start_idx = last_target_idx_;
        size_t end_idx = std::min(start_idx + forward_window, current_path_.size());

        // Search in forward window
        for (size_t i = start_idx; i < end_idx; ++i) {
            double dx = current_path_[i].x - px;
            double dy = current_path_[i].y - py;
            double dist = std::hypot(dx, dy);

            if (dist < result.distance) {
                result.distance = dist;
                result.idx = i;
                result.x = current_path_[i].x;
                result.y = current_path_[i].y;
                result.yaw = current_path_[i].yaw;
            }
        }

        // If no valid point found in forward window (e.g., vehicle deviated significantly),
        // fall back to full path search
        if (result.distance == std::numeric_limits<double>::infinity()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Forward tracking: No point found in forward window [%zu, %zu), "
                "falling back to full search", start_idx, end_idx);

            // Full path search
            for (size_t i = 0; i < current_path_.size(); ++i) {
                double dx = current_path_[i].x - px;
                double dy = current_path_[i].y - py;
                double dist = std::hypot(dx, dy);

                if (dist < result.distance) {
                    result.distance = dist;
                    result.idx = i;
                    result.x = current_path_[i].x;
                    result.y = current_path_[i].y;
                    result.yaw = current_path_[i].yaw;
                }
            }
        }

        // Update last target index for next cycle
        last_target_idx_ = result.idx;

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Forward tracking: search window [%zu, %zu), found idx=%zu, dist=%.3fm",
            start_idx, end_idx, result.idx, result.distance);

    } else {
        // Standard mode: Search entire path
        for (size_t i = 0; i < current_path_.size(); ++i) {
            double dx = current_path_[i].x - px;
            double dy = current_path_[i].y - py;
            double dist = std::hypot(dx, dy);

            if (dist < result.distance) {
                result.distance = dist;
                result.idx = i;
                result.x = current_path_[i].x;
                result.y = current_path_[i].y;
                result.yaw = current_path_[i].yaw;
            }
        }
    }

    return result;
}

double FrenetFollowerNode::computeOptimizedSteering(double px, double py, double vehicle_yaw,
                                                      const ClosestPointResult& closest,
                                                      double velocity) {
    // =============================================================================
    // OPTIMIZED STEERING CONTROL LAW
    // =============================================================================
    // δ = k_h * e_heading + k_l(v) * e_lateral + k_κ * κ_path
    //
    // Where:
    // - e_heading: Heading error (path_yaw - vehicle_yaw)
    // - e_lateral: Cross-track error (perpendicular distance from path)
    // - κ_path: Path curvature at target point
    // - k_l(v): Velocity-adaptive lateral gain = k_lateral / (1 + k_lateral_velocity * v)
    // =============================================================================

    // COMPONENT 1: Heading Error Control
    // -----------------------------------------------------------------------------
    // Aligns vehicle heading with desired path heading
    // High k_heading → aggressive heading alignment, may oscillate
    // Low k_heading → smooth but slower heading convergence
    double heading_error = computeHeadingError(vehicle_yaw, closest.yaw);
    double heading_term = k_heading_ * heading_error;

    // COMPONENT 2: Lateral Error Feedback (Stanley-inspired)
    // -----------------------------------------------------------------------------
    // Steers toward path centerline to reduce cross-track error
    // Uses velocity-adaptive gain to maintain stability at high speeds
    // Formula: k_l(v) = k_lateral / (1 + k_lateral_velocity * v)
    double lateral_error = computeLateralError(px, py, closest);
    double safe_velocity = std::max(0.1, std::abs(velocity));
    double adaptive_lateral_gain = k_lateral_ / (1.0 + k_lateral_velocity_ * safe_velocity);
    double lateral_term = std::atan(adaptive_lateral_gain * lateral_error / safe_velocity);

    // COMPONENT 3: Curvature Feedforward
    // -----------------------------------------------------------------------------
    // Anticipates path curvature for proactive steering
    // Reduces tracking lag in corners by pre-steering based on path geometry
    // Positive curvature = left turn, negative = right turn
    double curvature_term = 0.0;
    if (use_curvature_feedforward_) {
        double path_curvature = current_path_[closest.idx].kappa;
        // Convert curvature to steering angle using bicycle model
        // δ ≈ L * κ for small angles, where L = wheelbase
        curvature_term = k_curvature_ * wheelbase_ * path_curvature;
    }

    // TOTAL STEERING: Sum of three components
    double total_steering = heading_term + lateral_term + curvature_term;

    // Debug logging
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Steering components: heading=%.3f (err=%.3f), lateral=%.3f (err=%.3f), "
        "curvature=%.3f (κ=%.4f) → total=%.3f rad",
        heading_term, heading_error, lateral_term, lateral_error,
        curvature_term, current_path_[closest.idx].kappa, total_steering);

    return total_steering;
}

double FrenetFollowerNode::computeHeadingError(double vehicle_yaw, double path_yaw) {
    double error = path_yaw - vehicle_yaw;

    // Normalize to [-π, π]
    while (error > M_PI) error -= 2.0 * M_PI;
    while (error < -M_PI) error += 2.0 * M_PI;

    return error;
}

double FrenetFollowerNode::computeLateralError(double px, double py,
                                                const ClosestPointResult& closest) {
    if (current_path_.empty() || closest.idx >= current_path_.size()) {
        return 0.0;
    }

    // Vector from vehicle to closest point
    double dx = closest.x - px;
    double dy = closest.y - py;

    // Project onto path perpendicular direction
    // Lateral error = signed perpendicular distance
    double lateral_error = -dx * std::sin(closest.yaw) + dy * std::cos(closest.yaw);

    return lateral_error;
}

double FrenetFollowerNode::computeCurvatureAtPoint(size_t idx) {
    if (current_path_.size() < 3) {
        return 0.0;
    }

    size_t prev_idx = (idx > 0) ? idx - 1 : 0;
    size_t curr_idx = std::min(idx, current_path_.size() - 1);
    size_t next_idx = std::min(idx + 1, current_path_.size() - 1);

    return computeCurvature(current_path_[prev_idx],
                           current_path_[curr_idx],
                           current_path_[next_idx]);
}

double FrenetFollowerNode::computeCurvature(const PathPoint& p1,
                                             const PathPoint& p2,
                                             const PathPoint& p3) {
    // Three-point curvature estimation
    // κ = 2 * |cross(v1, v2)| / |v1|³
    double v1x = p2.x - p1.x;
    double v1y = p2.y - p1.y;
    double v2x = p3.x - p2.x;
    double v2y = p3.y - p2.y;

    double cross = v1x * v2y - v1y * v2x;
    double v1_mag = std::hypot(v1x, v1y);

    if (v1_mag < 1e-6) {
        return 0.0;
    }

    double kappa = 2.0 * cross / (v1_mag * v1_mag * v1_mag);

    return kappa;
}

double FrenetFollowerNode::applySteeringFilter(double raw_steering) {
    if (!use_steering_filter_) {
        return raw_steering;
    }

    // Low-pass filter: δ_filtered = (1 - α) * δ_prev + α * δ_raw
    double filtered_steering = (1.0 - steering_alpha_) * prev_steering_ +
                               steering_alpha_ * raw_steering;

    prev_steering_ = filtered_steering;

    return filtered_steering;
}

// =============================================================================
// SPEED CONTROL FUNCTIONS (from path_tracker)
// =============================================================================

double FrenetFollowerNode::computeSpeed(const PathPoint& target_point, double current_speed) {
    double speed = default_speed_;

    switch (speed_mode_) {
        case SpeedMode::PATH_VELOCITY:
            speed = (target_point.v > 0.01) ? target_point.v : default_speed_;
            break;

        case SpeedMode::CURVATURE_BASED:
            speed = computeCurvatureSpeed(0);  // Use current position curvature
            break;

        case SpeedMode::OPTIMIZE: {
            double path_speed = (target_point.v > 0.01) ? target_point.v : default_speed_;
            double curv_speed = computeCurvatureSpeed(0);
            speed = std::min(path_speed, curv_speed);
            break;
        }

        case SpeedMode::DEFAULT:
        default:
            speed = default_speed_;
            break;
    }

    // Clamp to limits
    speed = std::max(min_speed_limit_, std::min(max_speed_limit_, speed));

    // Apply debug mode adjustments
    if (debug_mode_) {
        speed *= velocity_gain_;
        speed = std::max(speed, debug_min_speed_);
    }

    return speed;
}

double FrenetFollowerNode::getGlobalPathSpeed(double x, double y) {
    if (!global_path_received_ || global_path_.empty()) {
        return default_speed_;
    }

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

    return global_path_[closest_idx].v;
}

double FrenetFollowerNode::computeCurvatureSpeed(size_t idx) {
    if (current_path_.size() < 3) {
        return default_speed_;
    }

    size_t curr_idx = std::min(idx, current_path_.size() - 1);
    double kappa = std::abs(current_path_[curr_idx].kappa);

    if (kappa < 1e-6) {
        return max_speed_limit_;
    }

    const double g = 9.81;
    double v_max = std::sqrt(friction_coeff_ * g / kappa);

    return v_max;
}

// =============================================================================
// ACCELERATION LIMITING FUNCTIONS (from path_tracker)
// =============================================================================

bool FrenetFollowerNode::loadLateralAccelerationLookupTable(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open lookup table: %s", filename.c_str());
        return false;
    }

    std::string line;
    std::vector<std::vector<double>> data;

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

    if (data.size() < 2 || data[0].size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Invalid lookup table dimensions");
        return false;
    }

    // Extract velocities from first row
    velocities_.clear();
    for (size_t i = 1; i < data[0].size(); ++i) {
        velocities_.push_back(data[0][i]);
    }

    // Extract steering angles and acceleration table
    steering_angles_.clear();
    accel_table_.clear();

    for (size_t i = 1; i < data.size(); ++i) {
        steering_angles_.push_back(data[i][0]);

        std::vector<double> accel_row;
        for (size_t j = 1; j < data[i].size(); ++j) {
            accel_row.push_back(data[i][j]);
        }
        accel_table_.push_back(accel_row);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded lookup table: %zu x %zu",
                steering_angles_.size(), velocities_.size());

    return true;
}

double FrenetFollowerNode::getMaxLateralAccelerationFromTable(double steering_angle,
                                                                double current_speed) {
    if (!use_acceleration_limit_ || steering_angles_.empty() || velocities_.empty()) {
        return max_total_acceleration_;
    }

    // Use absolute steering angle
    double abs_steering = std::abs(steering_angle);

    // Find closest steering angles
    size_t lower_steer_idx = 0;
    size_t upper_steer_idx = 0;

    for (size_t i = 0; i < steering_angles_.size(); ++i) {
        if (steering_angles_[i] <= abs_steering) {
            lower_steer_idx = i;
        } else {
            upper_steer_idx = i;
            break;
        }
    }

    if (upper_steer_idx == 0 && abs_steering > steering_angles_.back()) {
        lower_steer_idx = steering_angles_.size() - 1;
        upper_steer_idx = steering_angles_.size() - 1;
    }

    // Find closest velocities
    size_t lower_vel_idx = 0;
    size_t upper_vel_idx = 0;

    for (size_t i = 0; i < velocities_.size(); ++i) {
        if (velocities_[i] <= current_speed) {
            lower_vel_idx = i;
        } else {
            upper_vel_idx = i;
            break;
        }
    }

    if (upper_vel_idx == 0 && current_speed > velocities_.back()) {
        lower_vel_idx = velocities_.size() - 1;
        upper_vel_idx = velocities_.size() - 1;
    }

    // Bilinear interpolation
    double accel_00 = (lower_steer_idx < accel_table_.size() &&
                      lower_vel_idx < accel_table_[lower_steer_idx].size())
                      ? accel_table_[lower_steer_idx][lower_vel_idx] : 0.0;
    double accel_10 = (upper_steer_idx < accel_table_.size() &&
                      lower_vel_idx < accel_table_[upper_steer_idx].size())
                      ? accel_table_[upper_steer_idx][lower_vel_idx] : 0.0;
    double accel_01 = (lower_steer_idx < accel_table_.size() &&
                      upper_vel_idx < accel_table_[lower_steer_idx].size())
                      ? accel_table_[lower_steer_idx][upper_vel_idx] : 0.0;
    double accel_11 = (upper_steer_idx < accel_table_.size() &&
                      upper_vel_idx < accel_table_[upper_steer_idx].size())
                      ? accel_table_[upper_steer_idx][upper_vel_idx] : 0.0;

    double accel;
    if (lower_steer_idx == upper_steer_idx && lower_vel_idx == upper_vel_idx) {
        accel = accel_00;
    } else if (lower_steer_idx == upper_steer_idx) {
        double t_vel = (current_speed - velocities_[lower_vel_idx]) /
                       (velocities_[upper_vel_idx] - velocities_[lower_vel_idx]);
        accel = accel_00 + t_vel * (accel_01 - accel_00);
    } else if (lower_vel_idx == upper_vel_idx) {
        double t_steer = (abs_steering - steering_angles_[lower_steer_idx]) /
                         (steering_angles_[upper_steer_idx] - steering_angles_[lower_steer_idx]);
        accel = accel_00 + t_steer * (accel_10 - accel_00);
    } else {
        double t_steer = (abs_steering - steering_angles_[lower_steer_idx]) /
                         (steering_angles_[upper_steer_idx] - steering_angles_[lower_steer_idx]);
        double t_vel = (current_speed - velocities_[lower_vel_idx]) /
                       (velocities_[upper_vel_idx] - velocities_[lower_vel_idx]);

        double accel_0 = accel_00 + t_steer * (accel_10 - accel_00);
        double accel_1 = accel_01 + t_steer * (accel_11 - accel_01);
        accel = accel_0 + t_vel * (accel_1 - accel_0);
    }

    return std::min(accel, max_total_acceleration_);
}

double FrenetFollowerNode::computeCurrentLateralAcceleration(double steering_angle,
                                                               double current_speed) {
    if (std::abs(current_speed) < 0.01) {
        return 0.0;
    }

    double curvature = std::tan(steering_angle) / wheelbase_;
    double lateral_accel = current_speed * current_speed * std::abs(curvature);

    return lateral_accel;
}

double FrenetFollowerNode::computeMaxLongitudinalAcceleration(double lateral_accel) {
    double a_total_sq = max_total_acceleration_ * max_total_acceleration_;
    double a_lat_sq = lateral_accel * lateral_accel;

    if (a_lat_sq >= a_total_sq) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Lateral accel exceeds limit!");
        return 0.0;
    }

    return std::sqrt(a_total_sq - a_lat_sq);
}

double FrenetFollowerNode::applyAccelerationLimit(double target_speed, double current_speed,
                                                    double dt, double steering_angle) {
    double desired_accel = (target_speed - current_speed) / dt;
    double lateral_accel = computeCurrentLateralAcceleration(steering_angle, current_speed);
    double max_long_accel = computeMaxLongitudinalAcceleration(lateral_accel);

    double limited_accel = std::max(-max_long_accel, std::min(max_long_accel, desired_accel));
    double limited_speed = current_speed + limited_accel * dt;

    limited_speed = std::max(min_speed_limit_, std::min(max_speed_limit_, limited_speed));

    return limited_speed;
}

// =============================================================================
// PATH INTERPOLATION (from path_tracker)
// =============================================================================

void FrenetFollowerNode::interpolatePath(const std::vector<PathPoint>& coarse_path,
                                          std::vector<PathPoint>& fine_path,
                                          double resolution) {
    fine_path.clear();

    if (coarse_path.size() < 2) {
        fine_path = coarse_path;
        return;
    }

    fine_path.reserve(coarse_path.size() * 5);

    for (size_t i = 0; i < coarse_path.size() - 1; ++i) {
        const PathPoint& p1 = coarse_path[i];
        const PathPoint& p2 = coarse_path[i + 1];

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double segment_length = std::hypot(dx, dy);

        int num_points = static_cast<int>(std::ceil(segment_length / resolution));
        num_points = std::max(1, num_points);

        for (int j = 0; j < num_points; ++j) {
            double t = static_cast<double>(j) / num_points;

            PathPoint pt;
            pt.x = p1.x + t * dx;
            pt.y = p1.y + t * dy;

            // Interpolate yaw
            double dyaw = p2.yaw - p1.yaw;
            while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
            pt.yaw = p1.yaw + t * dyaw;

            // Interpolate velocity
            pt.v = p1.v + t * (p2.v - p1.v);
            pt.kappa = 0.0;

            fine_path.push_back(pt);
        }
    }

    fine_path.push_back(coarse_path.back());
}

// =============================================================================
// VISUALIZATION
// =============================================================================

void FrenetFollowerNode::visualizeTargetPoint(double x, double y) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "frenet_target";
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

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    target_pub_->publish(marker);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrenetFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
