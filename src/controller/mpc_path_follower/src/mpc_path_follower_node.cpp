#include "mpc_path_follower/mpc_path_follower_node.hpp"

using namespace std::chrono_literals;

MPCPathFollowerNode::MPCPathFollowerNode()
    : Node("mpc_path_follower"),
      path_received_(false) {

    // Declare and get parameters
    this->declare_parameter<int>("prediction_horizon", 10);
    this->declare_parameter<double>("dt", 0.1);

    // Vehicle parameters
    this->declare_parameter<double>("wheelbase", 0.33);
    this->declare_parameter<double>("max_steering_angle", 0.4189);
    this->declare_parameter<double>("max_acceleration", 4.0);
    this->declare_parameter<double>("max_deceleration", 6.0);
    this->declare_parameter<double>("max_speed", 8.0);
    this->declare_parameter<double>("min_speed", 0.5);

    // Cost weights
    this->declare_parameter<double>("weight_lateral_error", 10.0);
    this->declare_parameter<double>("weight_heading_error", 5.0);
    this->declare_parameter<double>("weight_velocity_error", 1.0);
    this->declare_parameter<double>("weight_steering", 0.1);
    this->declare_parameter<double>("weight_acceleration", 0.1);
    this->declare_parameter<double>("weight_steering_rate", 1.0);
    this->declare_parameter<double>("weight_acceleration_rate", 0.5);

    // Topics
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("path_topic", "/frenet_path");
    this->declare_parameter<std::string>("base_frame", "base_link");

    // Other parameters
    this->declare_parameter<double>("path_timeout", 1.0);
    this->declare_parameter<bool>("debug_mode", false);
    this->declare_parameter<double>("velocity_gain", 1.0);

    // Build MPC configuration
    MPCController::Config mpc_config;
    mpc_config.prediction_horizon = this->get_parameter("prediction_horizon").as_int();
    mpc_config.dt = this->get_parameter("dt").as_double();
    mpc_config.wheelbase = this->get_parameter("wheelbase").as_double();
    mpc_config.max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    mpc_config.max_acceleration = this->get_parameter("max_acceleration").as_double();
    mpc_config.max_deceleration = this->get_parameter("max_deceleration").as_double();
    mpc_config.max_speed = this->get_parameter("max_speed").as_double();
    mpc_config.min_speed = this->get_parameter("min_speed").as_double();
    mpc_config.weight_lateral_error = this->get_parameter("weight_lateral_error").as_double();
    mpc_config.weight_heading_error = this->get_parameter("weight_heading_error").as_double();
    mpc_config.weight_velocity_error = this->get_parameter("weight_velocity_error").as_double();
    mpc_config.weight_steering = this->get_parameter("weight_steering").as_double();
    mpc_config.weight_acceleration = this->get_parameter("weight_acceleration").as_double();
    mpc_config.weight_steering_rate = this->get_parameter("weight_steering_rate").as_double();
    mpc_config.weight_acceleration_rate = this->get_parameter("weight_acceleration_rate").as_double();

    // Get other parameters
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    path_timeout_ = this->get_parameter("path_timeout").as_double();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    velocity_gain_ = this->get_parameter("velocity_gain").as_double();

    // Create MPC controller
    mpc_controller_ = std::make_unique<MPCController>(mpc_config);

    // QoS settings
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos,
        std::bind(&MPCPathFollowerNode::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, path_qos,
        std::bind(&MPCPathFollowerNode::pathCallback, this, std::placeholders::_1));

    // Publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, control_qos);

    predicted_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "mpc_predicted_path", viz_qos);

    RCLCPP_INFO(this->get_logger(), "MPC Path Follower Node initialized");
    RCLCPP_INFO(this->get_logger(), "Prediction horizon: %d steps, dt: %.3f s",
                mpc_config.prediction_horizon, mpc_config.dt);
    RCLCPP_INFO(this->get_logger(), "Listening to path topic: %s", path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to drive topic: %s", drive_topic_.c_str());
}

void MPCPathFollowerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Received empty path");
        return;
    }

    // Convert path message to MPC states
    current_path_.clear();
    path_velocities_.clear();

    for (const auto& pose : msg->poses) {
        MPCController::State state;
        state.x = pose.pose.position.x;
        state.y = pose.pose.position.y;
        state.yaw = quatToYaw(pose.pose.orientation);

        // Extract velocity from pose.position.z (same as path_tracker)
        double velocity = (pose.pose.position.z > 0.01) ?
                         pose.pose.position.z : 2.0;  // default 2.0 m/s

        state.v = velocity;
        path_velocities_.push_back(velocity);

        current_path_.push_back(state);
    }

    last_path_time_ = this->now();
    path_received_ = true;

    auto min_v = *std::min_element(path_velocities_.begin(), path_velocities_.end());
    auto max_v = *std::max_element(path_velocities_.begin(), path_velocities_.end());

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Received path with %zu points | Velocity: min=%.2f, max=%.2f m/s",
                        current_path_.size(), min_v, max_v);
}

void MPCPathFollowerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Check if path is available
    if (!path_received_ || current_path_.size() < 2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "No valid path available for MPC tracking");
        publishStopCommand();
        return;
    }

    // Check path timeout
    double path_age = (this->now() - last_path_time_).seconds();
    if (path_age > path_timeout_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Path is stale (%.2f s old), stopping", path_age);
        publishStopCommand();
        return;
    }

    // Get current state from odometry
    MPCController::State current_state;
    current_state.x = msg->pose.pose.position.x;
    current_state.y = msg->pose.pose.position.y;
    current_state.yaw = quatToYaw(msg->pose.pose.orientation);
    current_state.v = std::hypot(msg->twist.twist.linear.x,
                                 msg->twist.twist.linear.y);

    // Solve MPC problem
    auto solution = mpc_controller_->solve(current_state, current_path_, path_velocities_);

    if (!solution.success || solution.control_sequence.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "MPC solver failed, stopping");
        publishStopCommand();
        return;
    }

    // Get first control command (receding horizon)
    auto control = solution.control_sequence[0];

    // Apply velocity gain for debugging
    double commanded_speed = current_state.v + control.acceleration * 0.1;  // dt = 0.1
    commanded_speed *= velocity_gain_;

    // Publish drive command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = base_frame_;
    drive_msg.drive.steering_angle = control.steering;
    drive_msg.drive.speed = commanded_speed;
    drive_msg.drive.acceleration = control.acceleration;

    drive_pub_->publish(drive_msg);

    // Visualize predicted trajectory
    if (debug_mode_) {
        visualizePredictedPath(solution.predicted_states);
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "MPC control: steering=%.3f rad, speed=%.2f m/s, accel=%.2f m/s²",
                         control.steering, commanded_speed, control.acceleration);
}

void MPCPathFollowerNode::publishStopCommand() {
    ackermann_msgs::msg::AckermannDriveStamped stop_msg;
    stop_msg.header.stamp = this->now();
    stop_msg.header.frame_id = base_frame_;
    stop_msg.drive.steering_angle = 0.0;
    stop_msg.drive.speed = 0.0;
    stop_msg.drive.acceleration = 0.0;
    drive_pub_->publish(stop_msg);
}

void MPCPathFollowerNode::visualizePredictedPath(
    const std::vector<MPCController::State>& predicted_states) {

    visualization_msgs::msg::MarkerArray marker_array;

    // Line strip for predicted path
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = "mpc_predicted_path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;  // Line width
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    for (const auto& state : predicted_states) {
        geometry_msgs::msg::Point p;
        p.x = state.x;
        p.y = state.y;
        p.z = 0.05;
        line_marker.points.push_back(p);
    }

    marker_array.markers.push_back(line_marker);

    // Spheres for predicted waypoints
    for (size_t i = 0; i < predicted_states.size(); ++i) {
        visualization_msgs::msg::Marker sphere_marker;
        sphere_marker.header.frame_id = "map";
        sphere_marker.header.stamp = this->now();
        sphere_marker.ns = "mpc_predicted_points";
        sphere_marker.id = i + 1;
        sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::msg::Marker::ADD;

        sphere_marker.pose.position.x = predicted_states[i].x;
        sphere_marker.pose.position.y = predicted_states[i].y;
        sphere_marker.pose.position.z = 0.05;
        sphere_marker.pose.orientation.w = 1.0;

        sphere_marker.scale.x = 0.1;
        sphere_marker.scale.y = 0.1;
        sphere_marker.scale.z = 0.1;

        sphere_marker.color.r = 0.0;
        sphere_marker.color.g = 1.0;
        sphere_marker.color.b = 1.0;
        sphere_marker.color.a = 0.8;

        marker_array.markers.push_back(sphere_marker);
    }

    predicted_path_pub_->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCPathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
