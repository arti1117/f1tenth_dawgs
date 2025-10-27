#include "lqr_path_follower/lqr_path_follower_node.hpp"

using namespace std::chrono_literals;

LQRPathFollowerNode::LQRPathFollowerNode()
    : Node("lqr_path_follower"),
      path_received_(false) {

    // Declare and get parameters
    this->declare_parameter<double>("dt", 0.05);
    this->declare_parameter<double>("wheelbase", 0.33);
    this->declare_parameter<double>("max_steering_angle", 0.4189);
    this->declare_parameter<double>("max_speed", 8.0);
    this->declare_parameter<double>("min_speed", 0.5);

    // LQR cost weights
    this->declare_parameter<double>("Q_lateral", 10.0);
    this->declare_parameter<double>("Q_heading", 5.0);
    this->declare_parameter<double>("Q_velocity", 1.0);
    this->declare_parameter<double>("R_steering", 1.0);
    this->declare_parameter<double>("R_steering_rate", 5.0);

    // Lookahead parameters
    this->declare_parameter<double>("lookahead_time", 0.8);
    this->declare_parameter<double>("min_lookahead_dist", 1.0);
    this->declare_parameter<double>("max_lookahead_dist", 3.0);

    // Topics
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("path_topic", "/frenet_path");
    this->declare_parameter<std::string>("base_frame", "base_link");

    // Other parameters
    this->declare_parameter<double>("path_timeout", 1.0);
    this->declare_parameter<bool>("debug_mode", false);
    this->declare_parameter<double>("velocity_gain", 1.0);

    // Build LQR configuration
    LQRController::Config lqr_config;
    lqr_config.wheelbase = this->get_parameter("wheelbase").as_double();
    lqr_config.dt = this->get_parameter("dt").as_double();
    lqr_config.max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    lqr_config.max_speed = this->get_parameter("max_speed").as_double();
    lqr_config.min_speed = this->get_parameter("min_speed").as_double();
    lqr_config.Q_lateral = this->get_parameter("Q_lateral").as_double();
    lqr_config.Q_heading = this->get_parameter("Q_heading").as_double();
    lqr_config.Q_velocity = this->get_parameter("Q_velocity").as_double();
    lqr_config.R_steering = this->get_parameter("R_steering").as_double();
    lqr_config.R_steering_rate = this->get_parameter("R_steering_rate").as_double();
    lqr_config.lookahead_time = this->get_parameter("lookahead_time").as_double();
    lqr_config.min_lookahead_dist = this->get_parameter("min_lookahead_dist").as_double();
    lqr_config.max_lookahead_dist = this->get_parameter("max_lookahead_dist").as_double();

    // Get other parameters
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    path_timeout_ = this->get_parameter("path_timeout").as_double();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    velocity_gain_ = this->get_parameter("velocity_gain").as_double();

    // Create LQR controller
    lqr_controller_ = std::make_unique<LQRController>(lqr_config);

    // QoS settings
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos,
        std::bind(&LQRPathFollowerNode::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, path_qos,
        std::bind(&LQRPathFollowerNode::pathCallback, this, std::placeholders::_1));

    // Publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, control_qos);

    predicted_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "lqr_predicted_path", viz_qos);

    RCLCPP_INFO(this->get_logger(), "LQR Path Follower Node initialized");
    RCLCPP_INFO(this->get_logger(), "Control frequency: %.1f Hz (dt=%.3f s)",
                1.0 / lqr_config.dt, lqr_config.dt);
    RCLCPP_INFO(this->get_logger(), "Listening to path topic: %s", path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to drive topic: %s", drive_topic_.c_str());
}

void LQRPathFollowerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Received empty path");
        return;
    }

    // Convert path message to LQR states
    current_path_.clear();
    path_velocities_.clear();

    for (const auto& pose : msg->poses) {
        LQRController::State state;
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

void LQRPathFollowerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Check if path is available
    if (!path_received_ || current_path_.size() < 2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "No valid path available for LQR tracking");
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
    LQRController::State current_state;
    current_state.x = msg->pose.pose.position.x;
    current_state.y = msg->pose.pose.position.y;
    current_state.yaw = quatToYaw(msg->pose.pose.orientation);
    current_state.v = std::hypot(msg->twist.twist.linear.x,
                                 msg->twist.twist.linear.y);

    // Compute LQR control
    auto control = lqr_controller_->computeControl(current_state, current_path_, path_velocities_);

    // Apply velocity gain for debugging
    double commanded_speed = current_state.v + control.acceleration * 0.05;  // dt = 0.05
    commanded_speed *= velocity_gain_;

    // Publish drive command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = base_frame_;
    drive_msg.drive.steering_angle = -control.steering;  // Negate steering for correct direction
    drive_msg.drive.speed = commanded_speed;
    drive_msg.drive.acceleration = control.acceleration;

    drive_pub_->publish(drive_msg);

    // Visualize predicted trajectory
    if (debug_mode_) {
        visualizePredictedPath(lqr_controller_->getPredictedTrajectory());
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "LQR control: steering=%.3f rad, speed=%.2f m/s, accel=%.2f m/s²",
                         control.steering, commanded_speed, control.acceleration);
}

void LQRPathFollowerNode::publishStopCommand() {
    ackermann_msgs::msg::AckermannDriveStamped stop_msg;
    stop_msg.header.stamp = this->now();
    stop_msg.header.frame_id = base_frame_;
    stop_msg.drive.steering_angle = 0.0;
    stop_msg.drive.speed = 0.0;
    stop_msg.drive.acceleration = 0.0;
    drive_pub_->publish(stop_msg);
}

void LQRPathFollowerNode::visualizePredictedPath(
    const std::vector<LQRController::State>& predicted_states) {

    visualization_msgs::msg::MarkerArray marker_array;

    // Line strip for predicted path
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = "lqr_predicted_path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;  // Line width
    line_marker.color.r = 1.0;
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

    predicted_path_pub_->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LQRPathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
