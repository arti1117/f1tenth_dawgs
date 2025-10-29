// path_tracker_node.cpp

#include "path_tracker/path_tracker_node.hpp"
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

PathTrackerNode::PathTrackerNode() : Node("path_tracker"), path_received_(false), global_path_received_(false), has_drive_cmd_(false) {
    // Declare parameters
    this->declare_parameter<double>("lookahead_base", 1.5);
    this->declare_parameter<double>("lookahead_k", 0.3);
    this->declare_parameter<double>("wheelbase", 0.33);
    this->declare_parameter<double>("default_speed", 2.0);
    this->declare_parameter<double>("max_steering_angle", 0.4189);  // 24 degrees
    this->declare_parameter<double>("path_timeout", 1.0);
    this->declare_parameter<bool>("use_speed_lookahead", true);

    // Adaptive lookahead parameters
    this->declare_parameter<bool>("use_adaptive_lookahead", true);
    this->declare_parameter<double>("lookahead_min", 0.5);
    this->declare_parameter<double>("lookahead_max", 3.0);
    this->declare_parameter<double>("k_curvature", 0.5);
    this->declare_parameter<double>("k_error", 0.3);
    this->declare_parameter<double>("curvature_epsilon", 0.001);

    // Stanley controller parameters
    this->declare_parameter<bool>("use_stanley", true);
    this->declare_parameter<double>("stanley_k", 0.5);

    // Steering filter parameters
    this->declare_parameter<bool>("use_steering_filter", true);
    this->declare_parameter<double>("steering_alpha", 0.3);

    // Heading control parameters (explicit heading error term)
    this->declare_parameter<bool>("use_heading_control", false);
    this->declare_parameter<double>("k_heading", 1.0);

    // Forward tracking parameters (for low odom frequency)
    this->declare_parameter<bool>("use_forward_tracking", false);
    this->declare_parameter<double>("forward_search_range", 2.0);

    // Debug mode parameters
    this->declare_parameter<bool>("debug_mode", false);
    this->declare_parameter<double>("velocity_gain", 1.0);
    this->declare_parameter<double>("debug_min_speed", 0.5);

    // Simulation mode parameter
    this->declare_parameter<bool>("sim_mode", false);
    this->declare_parameter<std::string>("sim_odom", "/ego_racecar/odom");

    // Speed mode parameters
    this->declare_parameter<std::string>("speed_mode", "default");  // "default", "path_velocity", "curvature", "optimize"
    this->declare_parameter<double>("friction_coeff", 0.9);
    this->declare_parameter<double>("max_speed_limit", 8.0);
    this->declare_parameter<double>("min_speed_limit", 0.5);

    // Optimize mode parameters
    this->declare_parameter<bool>("optimize_use_weighted", false);
    this->declare_parameter<double>("optimize_path_weight", 0.5);
    this->declare_parameter<double>("optimize_curv_weight", 0.5);

    // Position compensation parameters
    this->declare_parameter<bool>("use_position_compensation", true);
    this->declare_parameter<double>("expected_computation_time", 0.01);  // 10ms expected computation time

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

    // Adaptive lookahead parameters
    use_adaptive_lookahead_ = this->get_parameter("use_adaptive_lookahead").as_bool();
    lookahead_min_ = this->get_parameter("lookahead_min").as_double();
    lookahead_max_ = this->get_parameter("lookahead_max").as_double();
    k_curvature_ = this->get_parameter("k_curvature").as_double();
    k_error_ = this->get_parameter("k_error").as_double();
    curvature_epsilon_ = this->get_parameter("curvature_epsilon").as_double();

    // Stanley controller parameters
    use_stanley_ = this->get_parameter("use_stanley").as_bool();
    stanley_k_ = this->get_parameter("stanley_k").as_double();

    // Steering filter parameters
    use_steering_filter_ = this->get_parameter("use_steering_filter").as_bool();
    steering_alpha_ = this->get_parameter("steering_alpha").as_double();
    prev_steering_ = 0.0;  // Initialize previous steering for filter
    prev_steering_for_accel_ = 0.0;  // Initialize previous steering for acceleration

    // Heading control parameters
    use_heading_control_ = this->get_parameter("use_heading_control").as_bool();
    k_heading_ = this->get_parameter("k_heading").as_double();

    // Forward tracking parameters
    use_forward_tracking_ = this->get_parameter("use_forward_tracking").as_bool();
    forward_search_range_ = this->get_parameter("forward_search_range").as_double();
    last_target_idx_ = 0;  // Initialize last target index

    // Debug mode parameters
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    velocity_gain_ = this->get_parameter("velocity_gain").as_double();
    debug_min_speed_ = this->get_parameter("debug_min_speed").as_double();

    // Simulation mode parameter
    sim_mode_ = this->get_parameter("sim_mode").as_bool();

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

    // Optimize mode parameters
    optimize_use_weighted_ = this->get_parameter("optimize_use_weighted").as_bool();
    optimize_path_weight_ = this->get_parameter("optimize_path_weight").as_double();
    optimize_curv_weight_ = this->get_parameter("optimize_curv_weight").as_double();

    // Position compensation parameters
    use_position_compensation_ = this->get_parameter("use_position_compensation").as_bool();
    expected_computation_time_ = this->get_parameter("expected_computation_time").as_double();

    // Acceleration limiting parameters (friction circle based)
    this->declare_parameter<bool>("use_acceleration_limit", false);
    this->declare_parameter<std::string>("lateral_accel_lookup_table", "dawgs_lookup_table.csv");
    this->declare_parameter<std::string>("package_share_dir", "/home/dawgs_nx/f1tenth_dawgs/src/controller/path_tracker/config");
    this->declare_parameter<double>("max_total_acceleration", 9.81);  // 1g default

    use_acceleration_limit_ = this->get_parameter("use_acceleration_limit").as_bool();
    max_total_acceleration_ = this->get_parameter("max_total_acceleration").as_double();
    prev_commanded_speed_ = 0.0;  // Initialize previous speed

    // Load lateral acceleration lookup table if enabled
    if (use_acceleration_limit_) {
        std::string lookup_table_file = this->get_parameter("lateral_accel_lookup_table").as_string();
        std::string package_share_dir = this->get_parameter("package_share_dir").as_string();
        std::string lookup_table_path = package_share_dir + "/" + lookup_table_file;

        if (!loadLateralAccelerationLookupTable(lookup_table_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load lateral acceleration lookup table, disabling acceleration limit");
            use_acceleration_limit_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Friction circle acceleration limiting enabled with table: %s",
                        lookup_table_file.c_str());
            RCLCPP_INFO(this->get_logger(), "Max total acceleration (friction circle): %.2f m/s²",
                        max_total_acceleration_);
        }
    }

    odom_topic_ = this->get_parameter("odom_topic").as_string();

    // Override odom_topic with sim_odom if sim_mode is enabled
    if (sim_mode_) {
        odom_topic_ = this->get_parameter("sim_odom").as_string();
        RCLCPP_INFO(this->get_logger(), "Simulation mode enabled, using sim_odom topic: %s", odom_topic_.c_str());
    }

    drive_topic_ = this->get_parameter("drive_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    global_path_topic_ = this->get_parameter("global_path_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    // QoS for sensor data: Best Effort for low latency
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    sensor_qos.best_effort();

    // QoS for path data: Reliable for data integrity
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    path_qos.reliable();

    // QoS for control commands: Best Effort for real-time response with larger buffer
    auto control_qos = rclcpp::QoS(rclcpp::KeepLast(15));
    control_qos.best_effort();

    // QoS for visualization: Best Effort, small buffer
    auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    viz_qos.best_effort();

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos, std::bind(&PathTrackerNode::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, path_qos, std::bind(&PathTrackerNode::pathCallback, this, std::placeholders::_1));

    // Subscribe to global path for velocity reference (if needed)
    if (speed_mode_ == SpeedMode::PATH_VELOCITY || speed_mode_ == SpeedMode::OPTIMIZE) {
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            global_path_topic_, path_qos, std::bind(&PathTrackerNode::globalPathCallback, this, std::placeholders::_1));
    }

    // Publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, control_qos);
    lookahead_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_point", viz_qos);

    RCLCPP_INFO(this->get_logger(), "Path Tracker Node initialized");
    RCLCPP_INFO(this->get_logger(), "Listening to odom topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Listening to path topic: %s", path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Speed mode: %s", speed_mode_str.c_str());
    if (speed_mode_ == SpeedMode::PATH_VELOCITY || speed_mode_ == SpeedMode::OPTIMIZE) {
        RCLCPP_INFO(this->get_logger(), "Listening to global path topic: %s", global_path_topic_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Lookahead: base=%.2f m, k=%.2f", lookahead_base_, lookahead_k_);
    RCLCPP_INFO(this->get_logger(), "Simulation mode: %s (steering will be %s)",
                sim_mode_ ? "ENABLED" : "disabled",
                sim_mode_ ? "INVERTED" : "normal");
    RCLCPP_INFO(this->get_logger(), "Debug mode: %s, Velocity gain: %.2f, Min speed: %.2f m/s",
                debug_mode_ ? "ENABLED" : "disabled", velocity_gain_, debug_min_speed_);
}

void PathTrackerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Received empty path");
        return;
    }

    // Convert path message to internal representation (coarse path)
    std::vector<PathPoint> coarse_path;
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
            RCLCPP_DEBUG(this->get_logger(),
                "PATH_CALLBACK: Point[%zu]: pose.z=%.2f → pt.v=%.2f m/s",
                i, pose.pose.position.z, pt.v);
        }

        coarse_path.push_back(pt);
    }

    // Interpolate path for smoother tracking with low-frequency updates
    // This densifies the path so tracker has more fine-grained waypoints
    current_path_.clear();
    interpolatePath(coarse_path, current_path_, 0.1);  // 10cm resolution

    last_path_time_ = this->now();
    path_received_ = true;

    auto min_v = (*std::min_element(current_path_.begin(), current_path_.end(),
        [](const PathPoint& a, const PathPoint& b) { return a.v < b.v; })).v;
    auto max_v = (*std::max_element(current_path_.begin(), current_path_.end(),
        [](const PathPoint& a, const PathPoint& b) { return a.v < b.v; })).v;

    RCLCPP_INFO(this->get_logger(),
        "PATH_CALLBACK: Received path with %zu points → interpolated to %zu points (0.1m resolution) | "
        "Velocity stats: min=%.2f, max=%.2f m/s | Points with velocity: %d, Points using default: %d",
        coarse_path.size(), current_path_.size(), min_v, max_v,
        nonzero_velocity_count, zero_velocity_count);
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

        // Maintain last drive command if available, otherwise stop
        if (has_drive_cmd_) {
            last_drive_cmd_.header.stamp = this->now();
            drive_pub_->publish(last_drive_cmd_);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Maintaining last drive command while waiting for path (speed=%.2f, steering=%.3f)",
                last_drive_cmd_.drive.speed, last_drive_cmd_.drive.steering_angle);
        } else {
            // No previous command, must stop
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
            "Path is stale (%.2f s old), maintaining last drive command", path_age);

        // Maintain last drive command during path timeout (path planner is computing new path)
        if (has_drive_cmd_) {
            last_drive_cmd_.header.stamp = this->now();
            drive_pub_->publish(last_drive_cmd_);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Publishing buffered drive command (speed=%.2f, steering=%.3f)",
                last_drive_cmd_.drive.speed, last_drive_cmd_.drive.steering_angle);
        } else {
            // No previous command, must stop
            ackermann_msgs::msg::AckermannDriveStamped stop_msg;
            stop_msg.header.stamp = this->now();
            stop_msg.header.frame_id = base_frame_;
            stop_msg.drive.steering_angle = 0.0;
            stop_msg.drive.speed = 0.0;
            drive_pub_->publish(stop_msg);
        }
        return;
    }

    // Get current pose
    double px_raw = msg->pose.pose.position.x;
    double py_raw = msg->pose.pose.position.y;
    double yaw = quatToYaw(msg->pose.pose.orientation);
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double v = std::hypot(vx, vy);

    // Apply position compensation to account for latency
    double px = px_raw;
    double py = py_raw;

    if (use_position_compensation_ && v > 0.1) {  // Only compensate if moving
        // Calculate message delay (time since odometry was stamped)
        auto current_ros_time = this->now();
        auto odom_stamp = rclcpp::Time(msg->header.stamp);
        double message_delay = (current_ros_time - odom_stamp).seconds();

        // Estimate total delay (message delay + expected computation time)
        double total_lookahead_time = message_delay + expected_computation_time_;

        // Compensate position based on velocity and heading
        px = px_raw + v * std::cos(yaw) * total_lookahead_time;
        py = py_raw + v * std::sin(yaw) * total_lookahead_time;

        // Log position compensation
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Position compensation: delay=%.3fs (msg=%.3fs, comp=%.3fs), v=%.2fm/s, "
            "pos=(%.2f,%.2f)→(%.2f,%.2f), Δ=%.3fm",
            total_lookahead_time, message_delay, expected_computation_time_, v,
            px_raw, py_raw, px, py,
            std::hypot(px - px_raw, py - py_raw));
    }

    // =============================================================================
    // STEERING CALCULATION PROCESS (10-step pipeline)
    // =============================================================================
    // This controller combines Pure Pursuit (geometric path following) with
    // Stanley (lateral error correction) for accurate path tracking.
    //
    // Process Overview:
    // 1. Find closest point on path → reference for error calculation
    // 2. Calculate curvature → adapt lookahead for corners
    // 3. Calculate lateral error → measure how far off-path we are
    // 4. Compute base lookahead → speed-dependent forward looking distance
    // 5. Apply adaptive lookahead → adjust for curvature and error
    // 6. Find lookahead point → target point to steer towards
    // 7. Pure Pursuit steering → geometric steering to reach target
    // 8. Stanley correction → add lateral error feedback
    // 9. Apply low-pass filter → smooth out rapid steering changes
    // 10. Clamp to limits → enforce physical steering constraints
    // =============================================================================

    // STEP 1: Find closest point on path
    // -----------------------------------------------------------------------------
    // Locates the nearest waypoint on current_path_ to vehicle position (px, py).
    // This serves as the reference point for calculating lateral error and curvature.
    // Output: closest.idx (index), closest.distance (meters from path)
    auto closest = findClosestPoint(px, py);

    if (closest.distance > 5.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Vehicle too far from path: %.2f m", closest.distance);
        return;
    }

    // STEP 2: Calculate curvature at closest point
    // -----------------------------------------------------------------------------
    // Uses three-point finite difference method to estimate path curvature (kappa).
    // Formula: kappa = 2 * |cross(v1, v2)| / |v1|^3
    // where v1 = p2 - p1, v2 = p3 - p2
    // Effect: High curvature (tight turn) → reduce lookahead for sharper response
    //         Low curvature (straight) → increase lookahead for stability
    double curvature = computeCurvatureAtPoint(closest.idx);

    // STEP 3: Calculate lateral error (cross-track error)
    // -----------------------------------------------------------------------------
    // Computes perpendicular distance from vehicle to path at closest point.
    // Formula: e_y = -dx * sin(path_yaw) + dy * cos(path_yaw)
    // Sign convention: positive = right of path, negative = left of path
    // Used for: Adaptive lookahead adjustment + Stanley correction term
    double lateral_error = computeLateralError(px, py, closest);

    // STEP 4: Compute base lookahead distance
    // -----------------------------------------------------------------------------
    // Base lookahead distance determines how far ahead to look for target point.
    // Two modes:
    //   - Constant: L = lookahead_base (fixed distance)
    //   - Velocity-dependent: L = lookahead_base + lookahead_k * velocity
    // Effect: Higher speed → longer lookahead → smoother but less aggressive
    double base_lookahead = lookahead_base_;
    if (use_speed_lookahead_) {
        base_lookahead = lookahead_base_ + lookahead_k_ * std::max(0.0, v);
    }
    base_lookahead = std::max(0.5, base_lookahead);  // Minimum lookahead safety

    // STEP 5: Apply adaptive lookahead adjustment
    // -----------------------------------------------------------------------------
    // Dynamically adjusts lookahead based on path geometry and tracking error:
    //   - Curvature adjustment: L += k_curvature / (|kappa| + epsilon)
    //     → Increase lookahead in straight sections, reduce in curves
    //   - Error adjustment: L -= k_error * |lateral_error|
    //     → Reduce lookahead when far from path for quicker correction
    // Result: L_adaptive ∈ [lookahead_min, lookahead_max]
    double lookahead_dist = computeAdaptiveLookahead(base_lookahead, curvature, lateral_error, v);

    // STEP 6: Find lookahead point on path
    // -----------------------------------------------------------------------------
    // Walks along path from closest point, accumulating arc length until reaching
    // lookahead_dist meters ahead. Interpolates between waypoints for precision.
    // Output: lookahead.x, lookahead.y (target position), lookahead.v (target speed)
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

    // STEP 7: Compute Pure Pursuit steering angle
    // -----------------------------------------------------------------------------
    // Geometric path following algorithm that steers toward lookahead point.
    // Process:
    //   1. Transform lookahead point to vehicle frame (rotate by -yaw)
    //   2. Calculate angle to lookahead: alpha = atan2(y_veh, x_veh)
    //   3. Pure Pursuit formula: δ = atan(2 * L * sin(alpha) / lookahead_dist)
    // where L = wheelbase (distance between front and rear axles)
    // Result: Steering angle to make vehicle arc toward lookahead point
    double pure_pursuit_steering = computeSteeringAngle(px, py, yaw, lookahead.x, lookahead.y);

    // STEP 8: Add Stanley term for lateral error correction
    // -----------------------------------------------------------------------------
    // Stanley controller adds feedback based on cross-track error to prevent
    // corner cutting and improve path following accuracy.
    // Formula: δ_stanley = atan(k * e_y / v) + (path_yaw - vehicle_yaw)
    // Components:
    //   - Cross-track term: atan(k * lateral_error / velocity)
    //     → Steers toward path proportional to distance from centerline
    //   - Heading error: (path_yaw - vehicle_yaw)
    //     → Aligns vehicle heading with path direction
    // Effect: Corrects Pure Pursuit's tendency to cut corners in tight turns
    double path_yaw = current_path_[closest.idx].yaw;
    double stanley_correction = computeStanleyTerm(lateral_error, v, path_yaw, yaw);

    // STEP 8b: Optional explicit heading control
    // -----------------------------------------------------------------------------
    // Explicit heading error term for more direct heading alignment
    // Formula: δ_heading = k_heading * (path_yaw - vehicle_yaw)
    // Effect: More aggressive heading alignment than Stanley's implicit heading term
    // Enable when: Need stronger heading correction, especially at low speeds
    double heading_correction = 0.0;
    if (use_heading_control_) {
        // Normalize heading error to [-π, π]
        double heading_error = path_yaw - yaw;
        while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

        heading_correction = k_heading_ * heading_error;

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Heading control: error=%.3f rad, correction=%.3f rad",
            heading_error, heading_correction);
    }

    // STEP 9: Combine Pure Pursuit, Stanley, and optional Heading control
    // -----------------------------------------------------------------------------
    // Enhanced hybrid controller = geometric path following + error feedback + heading control
    // δ_combined = δ_pure_pursuit + δ_stanley + δ_heading
    // Result: Benefits of all approaches:
    //   - Pure Pursuit: Smooth geometric path following
    //   - Stanley: Tight tracking and corner-cutting suppression
    //   - Heading Control (optional): Stronger heading alignment
    double combined_steering = pure_pursuit_steering + stanley_correction + heading_correction;

    // STEP 10a: Apply low-pass filter for smooth steering
    // -----------------------------------------------------------------------------
    // First-order low-pass filter reduces high-frequency steering oscillations.
    // Formula: δ_filtered = (1 - α) * δ_prev + α * δ_raw
    // where α ∈ [0, 1] is the filter coefficient:
    //   - α = 1.0: No filtering (instant response, jerky motion)
    //   - α = 0.0: Maximum smoothing (slow response, stable)
    //   - Typical: α = 0.2-0.5 for balance
    // Effect: Smooths rapid steering changes → reduces mechanical wear and instability
    double filtered_steering = applySteeringFilter(combined_steering);

    // STEP 10b: Clamp steering angle to physical limits
    // -----------------------------------------------------------------------------
    // Enforces hard constraint: δ ∈ [-max_steering_angle, max_steering_angle]
    // Typical: max_steering_angle = 0.4189 rad (24 degrees) for F1TENTH
    // Safety: Prevents exceeding servo limits and vehicle stability limits
    double steering = std::max(-max_steering_angle_, std::min(max_steering_angle_, filtered_steering));

    // Compute speed based on selected mode
    double target_speed = computeSpeed(lookahead, v);

    // Apply acceleration limiting (if enabled) - friction circle based
    double commanded_speed = target_speed;
    double longitudinal_accel = 0.0;
    if (use_acceleration_limit_) {
        // Get time since last command (approximate with typical control loop time)
        double dt = 0.05;  // 20 Hz typical control loop, can be computed from timestamps

        // Apply friction circle acceleration rate limiting
        commanded_speed = applyAccelerationLimit(target_speed, v, dt, steering);
        longitudinal_accel = (commanded_speed - v) / dt;

        if (std::abs(commanded_speed - target_speed) > 0.1) {  // Log significant changes
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Speed limited by friction circle: target=%.2f → commanded=%.2f m/s (current=%.2f)",
                target_speed, commanded_speed, v);
        }
    }

    // Publish drive command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = base_frame_;
    drive_msg.drive.steering_angle = steering;
    drive_msg.drive.speed = commanded_speed;

    // Store longitudinal acceleration in the acceleration field
    if (use_acceleration_limit_) {
        drive_msg.drive.acceleration = longitudinal_accel;

        double lateral_accel = computeCurrentLateralAcceleration(steering, v);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Friction circle: a_lat=%.2f m/s², a_long=%.2f m/s², a_total=%.2f m/s² (steering=%.3f rad)",
            lateral_accel, longitudinal_accel,
            std::sqrt(lateral_accel * lateral_accel + longitudinal_accel * longitudinal_accel),
            steering);
    }

    drive_pub_->publish(drive_msg);

    // Store this command as the last valid drive command
    last_drive_cmd_ = drive_msg;
    has_drive_cmd_ = true;

    // Update previous commanded speed and steering for next cycle
    prev_commanded_speed_ = commanded_speed;
    prev_steering_for_accel_ = steering;

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

    if (current_path_.empty()) {
        return result;
    }

    // Forward tracking mode: search only ahead of last target
    // Benefit: Ensures sequential progression along planned_path when odom frequency is low
    if (use_forward_tracking_) {
        // Calculate forward search window size (based on path interpolation resolution ~0.1m)
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
            }
        }

        // Fallback: If no point found in forward window, search full path
        // This happens when vehicle deviates significantly or at path end
        if (result.distance == std::numeric_limits<double>::infinity()) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Forward tracking: No point in window [%zu, %zu), falling back to full search",
                start_idx, end_idx);

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
        }

        // Update last target index for next cycle
        last_target_idx_ = result.idx;

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Forward tracking: target_idx=%zu, search_range=[%zu, %zu), distance=%.3f",
            result.idx, start_idx, end_idx, result.distance);
    } else {
        // Standard mode: full path search
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
            // Calculate all speed components
            double path_speed;
            if (lookahead.v > 0.01) {
                path_speed = lookahead.v;
            } else {
                path_speed = getGlobalPathSpeed(lookahead.x, lookahead.y);
            }

            double curv_speed = computeCurvatureSpeed(lookahead.idx);

            // Choose between min() and weighted combination
            if (optimize_use_weighted_) {
                // Weighted combination approach
                // Normalize weights to sum to 1.0
                double weight_sum = optimize_path_weight_ + optimize_curv_weight_;
                if (weight_sum < 1e-6) {
                    // Safety: if both weights are zero, use equal weights
                    weight_sum = 1.0;
                    optimize_path_weight_ = 0.5;
                    optimize_curv_weight_ = 0.5;
                    RCLCPP_WARN_ONCE(this->get_logger(),
                        "Both optimize weights are zero, using equal weights (0.5, 0.5)");
                }

                double norm_path_weight = optimize_path_weight_ / weight_sum;
                double norm_curv_weight = optimize_curv_weight_ / weight_sum;

                speed = norm_path_weight * path_speed + norm_curv_weight * curv_speed;

                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "OPTIMIZE [WEIGHTED]: path=%.2f (w=%.2f), curv=%.2f (w=%.2f) → %.2f m/s",
                    path_speed, norm_path_weight, curv_speed, norm_curv_weight, speed);
            } else {
                // Original min() approach
                speed = std::min(path_speed, curv_speed);

                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "OPTIMIZE [MIN]: path=%.2f, curv=%.2f → %.2f m/s",
                    path_speed, curv_speed, speed);
            }
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

    // Apply velocity gain and minimum speed for debug mode
    double original_speed = speed;
    speed *= velocity_gain_;

    // Apply debug minimum speed if in debug mode
    if (debug_mode_) {
        speed = std::max(speed, debug_min_speed_);

        // Debug logging
        if (std::abs(velocity_gain_ - 1.0) > 0.01 || speed != original_speed * velocity_gain_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Debug mode: Original=%.2f m/s → After gain=%.2f m/s → Final=%.2f m/s (gain=%.2f, min=%.2f)",
                original_speed, original_speed * velocity_gain_, speed, velocity_gain_, debug_min_speed_);
        }
    }

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

bool PathTrackerNode::loadLateralAccelerationLookupTable(const std::string& filename) {
    // Open CSV file
    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open lateral acceleration lookup table: %s", filename.c_str());
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
                "Loaded lateral acceleration lookup table: %zu steering angles × %zu velocities",
                steering_angles_.size(), velocities_.size());

    return true;
}

double PathTrackerNode::getMaxLateralAccelerationFromTable(double steering_angle, double current_speed) {
    if (!use_acceleration_limit_ || steering_angles_.empty() || velocities_.empty()) {
        return max_total_acceleration_;  // Return max if not enabled
    }

    // Use absolute steering angle
    double abs_steering = std::abs(steering_angle);

    // Find the two closest steering angles in the lookup table
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

    // If beyond table range, use last index
    if (upper_steer_idx == 0 && abs_steering > steering_angles_.back()) {
        lower_steer_idx = steering_angles_.size() - 1;
        upper_steer_idx = steering_angles_.size() - 1;
    }

    // Find the two closest velocities in the lookup table
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

    // If beyond table range, use last index
    if (upper_vel_idx == 0 && current_speed > velocities_.back()) {
        lower_vel_idx = velocities_.size() - 1;
        upper_vel_idx = velocities_.size() - 1;
    }

    // Get four corner values for bilinear interpolation
    double accel_00 = (lower_steer_idx < accel_table_.size() && lower_vel_idx < accel_table_[lower_steer_idx].size())
                      ? accel_table_[lower_steer_idx][lower_vel_idx] : 0.0;
    double accel_10 = (upper_steer_idx < accel_table_.size() && lower_vel_idx < accel_table_[upper_steer_idx].size())
                      ? accel_table_[upper_steer_idx][lower_vel_idx] : 0.0;
    double accel_01 = (lower_steer_idx < accel_table_.size() && upper_vel_idx < accel_table_[lower_steer_idx].size())
                      ? accel_table_[lower_steer_idx][upper_vel_idx] : 0.0;
    double accel_11 = (upper_steer_idx < accel_table_.size() && upper_vel_idx < accel_table_[upper_steer_idx].size())
                      ? accel_table_[upper_steer_idx][upper_vel_idx] : 0.0;

    // Bilinear interpolation
    double accel;
    if (lower_steer_idx == upper_steer_idx && lower_vel_idx == upper_vel_idx) {
        // Exact match
        accel = accel_00;
    } else if (lower_steer_idx == upper_steer_idx) {
        // Interpolate in velocity dimension only
        double t_vel = (current_speed - velocities_[lower_vel_idx]) /
                       (velocities_[upper_vel_idx] - velocities_[lower_vel_idx]);
        accel = accel_00 + t_vel * (accel_01 - accel_00);
    } else if (lower_vel_idx == upper_vel_idx) {
        // Interpolate in steering dimension only
        double t_steer = (abs_steering - steering_angles_[lower_steer_idx]) /
                         (steering_angles_[upper_steer_idx] - steering_angles_[lower_steer_idx]);
        accel = accel_00 + t_steer * (accel_10 - accel_00);
    } else {
        // Full bilinear interpolation
        double t_steer = (abs_steering - steering_angles_[lower_steer_idx]) /
                         (steering_angles_[upper_steer_idx] - steering_angles_[lower_steer_idx]);
        double t_vel = (current_speed - velocities_[lower_vel_idx]) /
                       (velocities_[upper_vel_idx] - velocities_[lower_vel_idx]);

        double accel_0 = accel_00 + t_steer * (accel_10 - accel_00);
        double accel_1 = accel_01 + t_steer * (accel_11 - accel_01);
        accel = accel_0 + t_vel * (accel_1 - accel_0);
    }

    // Clamp to max total acceleration limit
    accel = std::min(accel, max_total_acceleration_);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Table max lateral accel: steering=%.3f rad, speed=%.2f m/s → a_lat_max=%.2f m/s²",
                          steering_angle, current_speed, accel);

    return accel;
}

double PathTrackerNode::computeCurrentLateralAcceleration(double steering_angle, double current_speed) {
    // Calculate actual lateral acceleration from current steering and speed
    // a_lateral = v² / R = v² * curvature
    // For Ackermann steering: curvature ≈ tan(steering_angle) / wheelbase
    // For small angles: curvature ≈ steering_angle / wheelbase

    if (std::abs(current_speed) < 0.01) {
        return 0.0;  // No lateral acceleration at standstill
    }

    double curvature = std::tan(steering_angle) / wheelbase_;
    double lateral_accel = current_speed * current_speed * std::abs(curvature);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Current lateral accel: v=%.2f m/s, steering=%.3f rad → a_lat=%.2f m/s²",
                          current_speed, steering_angle, lateral_accel);

    return lateral_accel;
}

double PathTrackerNode::computeMaxLongitudinalAcceleration(double lateral_accel) {
    // Friction circle constraint: a_total² = a_lateral² + a_longitudinal²
    // Therefore: a_longitudinal_max = sqrt(a_total_max² - a_lateral²)

    double a_total_sq = max_total_acceleration_ * max_total_acceleration_;
    double a_lat_sq = lateral_accel * lateral_accel;

    // Safety check: if lateral acceleration exceeds total limit, return 0
    if (a_lat_sq >= a_total_sq) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Lateral acceleration (%.2f m/s²) exceeds total limit (%.2f m/s²)! "
                             "Longitudinal acceleration set to 0",
                             lateral_accel, max_total_acceleration_);
        return 0.0;
    }

    double a_long_max = std::sqrt(a_total_sq - a_lat_sq);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Friction circle: a_lat=%.2f m/s² → a_long_max=%.2f m/s² (a_total_max=%.2f m/s²)",
                          lateral_accel, a_long_max, max_total_acceleration_);

    return a_long_max;
}

double PathTrackerNode::applyAccelerationLimit(double target_speed, double current_speed, double dt, double steering_angle) {
    // Calculate desired longitudinal acceleration
    double desired_accel = (target_speed - current_speed) / dt;

    // Calculate current lateral acceleration from steering and speed
    double lateral_accel = computeCurrentLateralAcceleration(steering_angle, current_speed);

    // Compute maximum longitudinal acceleration from friction circle
    double max_long_accel = computeMaxLongitudinalAcceleration(lateral_accel);

    // Limit acceleration to both positive (acceleration) and negative (braking)
    double limited_accel = std::max(-max_long_accel, std::min(max_long_accel, desired_accel));

    // Calculate limited speed
    double limited_speed = current_speed + limited_accel * dt;

    // Clamp to speed limits
    limited_speed = std::max(min_speed_limit_, std::min(max_speed_limit_, limited_speed));

    // Calculate total acceleration magnitude for validation
    double total_accel = std::sqrt(lateral_accel * lateral_accel + limited_accel * limited_accel);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Friction circle limit: desired_a_long=%.2f m/s² → limited_a_long=%.2f m/s² | "
                          "a_lat=%.2f m/s², a_total=%.2f m/s² (max=%.2f) | "
                          "target_speed=%.2f → limited_speed=%.2f m/s",
                          desired_accel, limited_accel, lateral_accel, total_accel, max_total_acceleration_,
                          target_speed, limited_speed);

    return limited_speed;
}

// ============= Adaptive Lookahead Functions =============

double PathTrackerNode::computeCurvatureAtPoint(size_t idx) {
    if (current_path_.size() < 3) {
        return 0.0;  // Not enough points
    }

    // Get three points for curvature calculation
    size_t prev_idx = (idx > 0) ? idx - 1 : 0;
    size_t curr_idx = std::min(idx, current_path_.size() - 1);
    size_t next_idx = std::min(idx + 1, current_path_.size() - 1);

    // Use existing computeCurvature function
    return computeCurvature(current_path_[prev_idx],
                           current_path_[curr_idx],
                           current_path_[next_idx]);
}

double PathTrackerNode::computeLateralError(double px, double py, const ClosestPointResult& closest) {
    if (current_path_.empty() || closest.idx >= current_path_.size()) {
        return 0.0;
    }

    // Vector from vehicle to closest point
    double dx = closest.x - px;
    double dy = closest.y - py;

    // Path direction at closest point
    double path_yaw = current_path_[closest.idx].yaw;

    // Project vehicle position onto path tangent to get lateral error
    // Lateral error = perpendicular distance (signed)
    double lateral_error = -dx * std::sin(path_yaw) + dy * std::cos(path_yaw);

    return lateral_error;
}

double PathTrackerNode::computeAdaptiveLookahead(double base_lookahead, double curvature,
                                                  double lateral_error, double velocity) {
    double adaptive_lookahead = base_lookahead;

    if (use_adaptive_lookahead_) {
        // Curvature-based adjustment: L = L_min + k_curv / (|curvature| + ε)
        double curv_adjustment = k_curvature_ / (std::abs(curvature) + curvature_epsilon_);

        // Lateral error-based adjustment: L = L0 - k_e * |e_y|
        double error_adjustment = -k_error_ * std::abs(lateral_error);

        // Combined adaptive lookahead
        adaptive_lookahead = lookahead_min_ + curv_adjustment + error_adjustment;

        // Clamp to min/max bounds
        adaptive_lookahead = std::max(lookahead_min_, std::min(lookahead_max_, adaptive_lookahead));

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Adaptive lookahead: base=%.2f, curv=%.4f, curv_adj=%.2f, error=%.3f, error_adj=%.2f → final=%.2f m",
            base_lookahead, curvature, curv_adjustment, lateral_error, error_adjustment, adaptive_lookahead);
    }

    return adaptive_lookahead;
}

// ============= Stanley Controller Functions =============

double PathTrackerNode::computeStanleyTerm(double lateral_error, double velocity,
                                           double path_yaw, double vehicle_yaw) {
    if (!use_stanley_) {
        return 0.0;
    }

    // Stanley term: δ_stanley = atan(k_e * e_y / v)
    // Avoid division by very small velocity
    double safe_velocity = std::max(0.1, std::abs(velocity));
    double stanley_term = std::atan(stanley_k_ * lateral_error / safe_velocity);

    // Also add heading error component
    double heading_error = path_yaw - vehicle_yaw;

    // Normalize heading error to [-π, π]
    while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

    // Total Stanley correction
    double total_stanley = stanley_term + heading_error;

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Stanley: lat_err=%.3f, v=%.2f, stanley_term=%.3f, heading_err=%.3f → total=%.3f",
        lateral_error, velocity, stanley_term, heading_error, total_stanley);

    return total_stanley;
}

double PathTrackerNode::applySteeringFilter(double raw_steering) {
    if (!use_steering_filter_) {
        return raw_steering;
    }

    // Low-pass filter: δ_filtered = (1 - α) * δ_prev + α * δ_raw
    double filtered_steering = (1.0 - steering_alpha_) * prev_steering_ + steering_alpha_ * raw_steering;

    // Update previous steering for next iteration
    prev_steering_ = filtered_steering;

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Steering filter: raw=%.3f, filtered=%.3f (α=%.2f)",
        raw_steering, filtered_steering, steering_alpha_);

    return filtered_steering;
}

void PathTrackerNode::interpolatePath(const std::vector<PathPoint>& coarse_path,
                                       std::vector<PathPoint>& fine_path,
                                       double resolution) {
    fine_path.clear();

    if (coarse_path.size() < 2) {
        fine_path = coarse_path;
        return;
    }

    // Reserve approximate space (overestimate is OK)
    fine_path.reserve(coarse_path.size() * 5);

    for (size_t i = 0; i < coarse_path.size() - 1; ++i) {
        const PathPoint& p1 = coarse_path[i];
        const PathPoint& p2 = coarse_path[i + 1];

        // Calculate distance between consecutive points
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double segment_length = std::hypot(dx, dy);

        // Calculate number of interpolated points needed
        int num_points = static_cast<int>(std::ceil(segment_length / resolution));
        num_points = std::max(1, num_points);

        // Add interpolated points
        for (int j = 0; j < num_points; ++j) {
            double t = static_cast<double>(j) / num_points;

            PathPoint pt;
            pt.x = p1.x + t * dx;
            pt.y = p1.y + t * dy;

            // Linear interpolation for yaw (handle angle wrapping)
            double dyaw = p2.yaw - p1.yaw;
            while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
            pt.yaw = p1.yaw + t * dyaw;

            // Linear interpolation for velocity
            pt.v = p1.v + t * (p2.v - p1.v);

            fine_path.push_back(pt);
        }
    }

    // Add the last point
    fine_path.push_back(coarse_path.back());

    RCLCPP_DEBUG(this->get_logger(),
        "Interpolated path: %zu coarse points → %zu fine points (resolution=%.2fm)",
        coarse_path.size(), fine_path.size(), resolution);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
