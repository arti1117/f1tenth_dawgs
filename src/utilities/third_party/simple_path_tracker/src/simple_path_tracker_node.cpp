#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

struct PathPoint {
    double x;
    double y;
    double v;      // velocity
    double kappa;  // curvature
};

class SimplePathTracker : public rclcpp::Node
{
public:
    SimplePathTracker() : Node("simple_path_tracker"),
                          tf_buffer_(this->get_clock()),
                          tf_listener_(tf_buffer_)
    {
        // Declare parameters
        declare_parameter<std::string>("csv_file_path", "");
        declare_parameter<std::string>("frame_id", "map");
        declare_parameter<std::string>("base_link_frame", "base_link");

        // Topics
        declare_parameter<std::string>("odom_topic", "/odom");
        declare_parameter<std::string>("scan_topic", "/scan");
        declare_parameter<std::string>("global_path_topic", "/global_path");
        declare_parameter<std::string>("local_path_topic", "/local_path");

        // Planning parameters
        declare_parameter<double>("lookahead_distance", 3.0);
        declare_parameter<double>("lookbehind_distance", 1.0);
        declare_parameter<int>("local_path_points", 50);
        declare_parameter<double>("path_resolution", 0.1);  // meters between points
        declare_parameter<double>("publish_rate", 20.0);    // Hz

        // Get parameters
        csv_file_path_ = get_parameter("csv_file_path").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        base_link_frame_ = get_parameter("base_link_frame").as_string();

        odom_topic_ = get_parameter("odom_topic").as_string();
        scan_topic_ = get_parameter("scan_topic").as_string();
        global_path_topic_ = get_parameter("global_path_topic").as_string();
        local_path_topic_ = get_parameter("local_path_topic").as_string();

        lookahead_distance_ = get_parameter("lookahead_distance").as_double();
        lookbehind_distance_ = get_parameter("lookbehind_distance").as_double();
        local_path_points_ = get_parameter("local_path_points").as_int();
        path_resolution_ = get_parameter("path_resolution").as_double();
        double publish_rate = get_parameter("publish_rate").as_double();

        // Load CSV path
        if (!csv_file_path_.empty()) {
            if (!loadPathFromCSV()) {
                RCLCPP_ERROR(get_logger(), "Failed to load path from CSV: %s", csv_file_path_.c_str());
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from CSV", global_path_.size());
        } else {
            RCLCPP_WARN(get_logger(), "No CSV file specified, waiting for global path topic");
        }

        // Create subscribers
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&SimplePathTracker::odomCallback, this, std::placeholders::_1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            std::bind(&SimplePathTracker::scanCallback, this, std::placeholders::_1));

        if (csv_file_path_.empty()) {
            global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
                global_path_topic_, 10,
                std::bind(&SimplePathTracker::globalPathCallback, this, std::placeholders::_1));
        }

        // Create publishers
        local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, 10);
        global_path_pub_ = create_publisher<nav_msgs::msg::Path>(global_path_topic_, 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", 10);

        // Create timer for main loop
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&SimplePathTracker::mainLoop, this));

        // Create timer for global path publishing (1 Hz)
        if (!csv_file_path_.empty()) {
            global_path_timer_ = create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&SimplePathTracker::publishGlobalPath, this));
        }

        RCLCPP_INFO(get_logger(), "Simple Path Tracker initialized");
    }

private:
    bool loadPathFromCSV()
    {
        std::ifstream file(csv_file_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Could not open CSV file: %s", csv_file_path_.c_str());
            return false;
        }

        global_path_.clear();
        std::string line;
        bool header_skipped = false;

        while (std::getline(file, line)) {
            // Skip header line if it contains non-numeric data
            if (!header_skipped && (line.find("x") != std::string::npos ||
                                   line.find("y") != std::string::npos)) {
                header_skipped = true;
                continue;
            }

            // Parse CSV line: x,y,v,kappa
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> cells;

            while (std::getline(ss, cell, ',')) {
                cells.push_back(cell);
            }

            if (cells.size() >= 2) {
                try {
                    PathPoint point;
                    point.x = std::stod(cells[0]);
                    point.y = std::stod(cells[1]);
                    point.v = (cells.size() >= 3) ? std::stod(cells[2]) : 1.0;
                    point.kappa = (cells.size() >= 4) ? std::stod(cells[3]) : 0.0;
                    global_path_.push_back(point);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "Failed to parse line: %s", line.c_str());
                    continue;
                }
            }
        }

        file.close();

        if (global_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid waypoints found in CSV file");
            return false;
        }

        // Build spatial index for fast nearest neighbor search
        buildSpatialIndex();

        return true;
    }

    void buildSpatialIndex()
    {
        // For a closed-loop track, ensure continuity
        if (global_path_.size() < 2) return;

        // Check if path is closed (first and last points are close)
        double dx = global_path_.front().x - global_path_.back().x;
        double dy = global_path_.front().y - global_path_.back().y;
        double dist = std::sqrt(dx * dx + dy * dy);

        is_closed_loop_ = (dist < 1.0);  // Consider closed if endpoints within 1m

        if (is_closed_loop_) {
            RCLCPP_INFO(get_logger(), "Detected closed-loop track");
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        have_odom_ = true;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
        have_scan_ = true;
    }

    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        global_path_.clear();
        for (const auto& pose_stamped : msg->poses) {
            PathPoint point;
            point.x = pose_stamped.pose.position.x;
            point.y = pose_stamped.pose.position.y;
            point.v = 1.0;  // Default velocity
            point.kappa = 0.0;  // Default curvature
            global_path_.push_back(point);
        }

        if (!global_path_.empty()) {
            buildSpatialIndex();
            RCLCPP_INFO(get_logger(), "Received global path with %zu waypoints", global_path_.size());
        }
    }

    int findNearestPointIndex(double x, double y)
    {
        if (global_path_.empty()) return -1;

        double min_dist = std::numeric_limits<double>::max();
        int nearest_idx = 0;

        for (size_t i = 0; i < global_path_.size(); ++i) {
            double dx = global_path_[i].x - x;
            double dy = global_path_[i].y - y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        return nearest_idx;
    }

    nav_msgs::msg::Path extractLocalPath(int nearest_idx)
    {
        nav_msgs::msg::Path local_path;
        local_path.header.frame_id = frame_id_;
        local_path.header.stamp = now();

        if (global_path_.empty() || nearest_idx < 0) {
            return local_path;
        }

        // Calculate start and end indices for local path
        int path_size = static_cast<int>(global_path_.size());

        // Calculate how many points we need based on distance and resolution
        int points_ahead = static_cast<int>(lookahead_distance_ / path_resolution_);
        int points_behind = static_cast<int>(lookbehind_distance_ / path_resolution_);

        // For closed-loop tracks, handle wrapping
        if (is_closed_loop_) {
            // Extract points with wrapping
            for (int i = -points_behind; i <= points_ahead; ++i) {
                int idx = (nearest_idx + i + path_size) % path_size;

                geometry_msgs::msg::PoseStamped pose;
                pose.header = local_path.header;
                pose.pose.position.x = global_path_[idx].x;
                pose.pose.position.y = global_path_[idx].y;
                pose.pose.position.z = 0.0;

                // Calculate orientation from path direction
                int next_idx = (idx + 1) % path_size;
                double dx = global_path_[next_idx].x - global_path_[idx].x;
                double dy = global_path_[next_idx].y - global_path_[idx].y;
                double yaw = std::atan2(dy, dx);

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = std::sin(yaw / 2.0);
                pose.pose.orientation.w = std::cos(yaw / 2.0);

                local_path.poses.push_back(pose);
            }
        } else {
            // For non-closed paths, clamp to valid indices
            int start_idx = std::max(0, nearest_idx - points_behind);
            int end_idx = std::min(path_size - 1, nearest_idx + points_ahead);

            for (int idx = start_idx; idx <= end_idx; ++idx) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = local_path.header;
                pose.pose.position.x = global_path_[idx].x;
                pose.pose.position.y = global_path_[idx].y;
                pose.pose.position.z = 0.0;

                // Calculate orientation
                if (idx < path_size - 1) {
                    double dx = global_path_[idx + 1].x - global_path_[idx].x;
                    double dy = global_path_[idx + 1].y - global_path_[idx].y;
                    double yaw = std::atan2(dy, dx);

                    pose.pose.orientation.x = 0.0;
                    pose.pose.orientation.y = 0.0;
                    pose.pose.orientation.z = std::sin(yaw / 2.0);
                    pose.pose.orientation.w = std::cos(yaw / 2.0);
                } else if (idx > 0) {
                    // Use same orientation as previous point for last point
                    pose.pose.orientation = local_path.poses.back().pose.orientation;
                } else {
                    // Default orientation
                    pose.pose.orientation.w = 1.0;
                }

                local_path.poses.push_back(pose);
            }
        }

        return local_path;
    }

    void publishVisualizationMarkers(int nearest_idx)
    {
        visualization_msgs::msg::MarkerArray markers;

        // Marker for nearest point
        visualization_msgs::msg::Marker nearest_marker;
        nearest_marker.header.frame_id = frame_id_;
        nearest_marker.header.stamp = now();
        nearest_marker.ns = "nearest_point";
        nearest_marker.id = 0;
        nearest_marker.type = visualization_msgs::msg::Marker::SPHERE;
        nearest_marker.action = visualization_msgs::msg::Marker::ADD;

        if (nearest_idx >= 0 && nearest_idx < static_cast<int>(global_path_.size())) {
            nearest_marker.pose.position.x = global_path_[nearest_idx].x;
            nearest_marker.pose.position.y = global_path_[nearest_idx].y;
            nearest_marker.pose.position.z = 0.5;
            nearest_marker.pose.orientation.w = 1.0;
            nearest_marker.scale.x = 0.3;
            nearest_marker.scale.y = 0.3;
            nearest_marker.scale.z = 0.3;
            nearest_marker.color.r = 1.0;
            nearest_marker.color.g = 0.0;
            nearest_marker.color.b = 0.0;
            nearest_marker.color.a = 1.0;
            markers.markers.push_back(nearest_marker);
        }

        // Marker for current position
        visualization_msgs::msg::Marker current_marker;
        current_marker.header.frame_id = frame_id_;
        current_marker.header.stamp = now();
        current_marker.ns = "current_position";
        current_marker.id = 0;
        current_marker.type = visualization_msgs::msg::Marker::ARROW;
        current_marker.action = visualization_msgs::msg::Marker::ADD;
        current_marker.pose = current_pose_;
        current_marker.scale.x = 0.5;
        current_marker.scale.y = 0.1;
        current_marker.scale.z = 0.1;
        current_marker.color.r = 0.0;
        current_marker.color.g = 1.0;
        current_marker.color.b = 0.0;
        current_marker.color.a = 1.0;
        markers.markers.push_back(current_marker);

        marker_pub_->publish(markers);
    }

    void publishGlobalPath()
    {
        if (global_path_.empty()) return;

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = frame_id_;
        path_msg.header.stamp = now();

        for (size_t i = 0; i < global_path_.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = global_path_[i].x;
            pose.pose.position.y = global_path_[i].y;
            pose.pose.position.z = 0.0;

            // Calculate orientation
            if (i < global_path_.size() - 1) {
                double dx = global_path_[i + 1].x - global_path_[i].x;
                double dy = global_path_[i + 1].y - global_path_[i].y;
                double yaw = std::atan2(dy, dx);

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = std::sin(yaw / 2.0);
                pose.pose.orientation.w = std::cos(yaw / 2.0);
            } else if (is_closed_loop_) {
                // For closed loop, connect last to first
                double dx = global_path_[0].x - global_path_[i].x;
                double dy = global_path_[0].y - global_path_[i].y;
                double yaw = std::atan2(dy, dx);

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = std::sin(yaw / 2.0);
                pose.pose.orientation.w = std::cos(yaw / 2.0);
            } else {
                // Use same as previous
                pose.pose.orientation = path_msg.poses.back().pose.orientation;
            }

            path_msg.poses.push_back(pose);
        }

        global_path_pub_->publish(path_msg);
    }

    void mainLoop()
    {
        if (!have_odom_ || global_path_.empty()) {
            return;
        }

        // Get current position
        double current_x = current_pose_.position.x;
        double current_y = current_pose_.position.y;

        // Find nearest point on global path
        int nearest_idx = findNearestPointIndex(current_x, current_y);

        if (nearest_idx < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Could not find nearest point on path");
            return;
        }

        // Extract local path
        nav_msgs::msg::Path local_path = extractLocalPath(nearest_idx);

        // Publish local path
        if (!local_path.poses.empty()) {
            local_path_pub_->publish(local_path);

            RCLCPP_DEBUG(get_logger(), "Published local path with %zu points (nearest idx: %d)",
                        local_path.poses.size(), nearest_idx);
        }

        // Publish visualization markers
        publishVisualizationMarkers(nearest_idx);
    }

    // ROS2 Communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr global_path_timer_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string csv_file_path_;
    std::string frame_id_;
    std::string base_link_frame_;
    std::string odom_topic_;
    std::string scan_topic_;
    std::string global_path_topic_;
    std::string local_path_topic_;

    double lookahead_distance_;
    double lookbehind_distance_;
    int local_path_points_;
    double path_resolution_;

    // State
    std::vector<PathPoint> global_path_;
    geometry_msgs::msg::Pose current_pose_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    bool have_odom_ = false;
    bool have_scan_ = false;
    bool is_closed_loop_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePathTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}