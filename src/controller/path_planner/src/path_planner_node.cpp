#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include <sstream>
#include <chrono>
#ifdef HAS_ACKERMANN
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#endif


#include "path_planner/frenet.hpp"
#include "path_planner/lattice_lut.hpp"
#include "path_planner/utils.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>


using std::placeholders::_1;


class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode(): Node("path_planner"), odom_received_(false), have_scan_(false)
    {
        declare_parameter<std::string>("csv_file_dir", "");
        declare_parameter<std::string>("csv_file_name", "");
        declare_parameter<std::string>("csv_file_path", ".../global_waypoints_iqp.csv");
        declare_parameter<bool>("sim_mode", false);
        declare_parameter<std::string>("sim_odom", "/ego_racecar/odom");
        declare_parameter<std::string>("odom_topic", "/odom");
        declare_parameter<std::string>("planned_path_topic", "/planned_path");
        declare_parameter<std::string>("global_path_topic", "/global_centerline");
        declare_parameter<std::string>("frenet_path_topic", "/frenet_path");
        declare_parameter<std::string>("lut_path_topic", "/lut_path");
        declare_parameter<std::string>("frame_id", "map");
        declare_parameter<bool>("use_lattice", true);
        declare_parameter<bool>("use_frenet", true);
        declare_parameter<bool>("visualize_paths", true);
        declare_parameter<double>("planner_horizon", 3.0);
        declare_parameter<double>("publish_rate", 1.0);
        declare_parameter<int>("log_level", 1); // 0=NONE, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG, 5=VERBOSE
        declare_parameter<std::string>("scan_topic", "/scan");
        declare_parameter<double>("expected_computation_time", 0.01); // Expected computation time in seconds
        declare_parameter<bool>("adaptive_compensation", true); // Enable adaptive computation time estimation

        // Frenet parameters
        declare_parameter<double>("frenet_time_horizon", 3.0);
        declare_parameter<double>("frenet_min_time", 1.0);
        declare_parameter<double>("frenet_target_speed", 3.0);
        declare_parameter<double>("frenet_dt", 0.05);
        declare_parameter<double>("frenet_max_speed", 15.0);
        declare_parameter<double>("frenet_max_accel", 4.0);
        declare_parameter<double>("frenet_lookahead_distance", 2.0);
        declare_parameter<std::vector<double>>("frenet_d_samples", std::vector<double>{-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0});
        declare_parameter<std::vector<double>>("frenet_t_samples", std::vector<double>{1.5, 2.0, 2.5, 3.0});
        declare_parameter<double>("frenet_k_jerk", 0.1);
        declare_parameter<double>("frenet_k_time", 0.1);
        declare_parameter<double>("frenet_k_deviation", 1.0);
        declare_parameter<double>("frenet_k_velocity", 1.0);
        declare_parameter<double>("frenet_safety_radius", 0.3);
        declare_parameter<double>("frenet_road_half_width", 1.2);

        // Obstacle detection parameters
        declare_parameter<double>("obstacle_cluster_distance", 0.5);
        declare_parameter<double>("obstacle_max_box_size", 1.0);
        declare_parameter<double>("obstacle_box_safety_margin", 0.4);

        // Enhanced safety parameters
        declare_parameter<double>("frenet_vehicle_radius", 0.2);
        declare_parameter<double>("frenet_obstacle_radius", 0.15);
        declare_parameter<double>("frenet_k_velocity_safety", 0.15);
        declare_parameter<double>("frenet_min_safety_margin", 0.25);
        declare_parameter<double>("frenet_k_proximity", 0.5);
        declare_parameter<double>("frenet_proximity_threshold", 1.5);
        declare_parameter<int>("frenet_interpolation_checks", 3);



        // QoS for sensor data: Best Effort for low latency
        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5));
        sensor_qos.best_effort();

        // QoS for path data: Reliable for data integrity
        auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        path_qos.reliable();

        // QoS for visualization: Best Effort, small buffer
        auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        viz_qos.best_effort();

        // Get odom topic, use sim_odom if sim_mode is enabled
        std::string odom_topic = get_parameter("odom_topic").as_string();
        bool sim_mode = get_parameter("sim_mode").as_bool();
        if (sim_mode) {
            odom_topic = get_parameter("sim_odom").as_string();
            RCLCPP_INFO(get_logger(), "Simulation mode enabled, using sim_odom topic: %s", odom_topic.c_str());
        }

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, sensor_qos,
        std::bind(&PathPlannerNode::odomCallback, this, _1));

        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(get_parameter("scan_topic").as_string(), sensor_qos,
        std::bind(&PathPlannerNode::scanCallback, this, _1));

        // Publishers
        pub_path_ = create_publisher<nav_msgs::msg::Path>(get_parameter("planned_path_topic").as_string(), path_qos);
        pub_global_path_ = create_publisher<nav_msgs::msg::Path>(get_parameter("global_path_topic").as_string(), path_qos);

        // Visualization publishers
        if (get_parameter("visualize_paths").as_bool()) {
            pub_frenet_path_ = create_publisher<nav_msgs::msg::Path>(get_parameter("frenet_path_topic").as_string(), path_qos);
            pub_lut_path_ = create_publisher<nav_msgs::msg::Path>(get_parameter("lut_path_topic").as_string(), path_qos);
            pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/path_planner_markers", viz_qos);
        }

        // Global path velocity visualization publisher
        pub_velocity_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/global_path_velocity_markers", viz_qos);

        // Frenet path velocity visualization publisher
        pub_frenet_velocity_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/frenet_path_velocity_markers", viz_qos);

        // Obstacle visualization publisher
        pub_obstacle_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_boxes", viz_qos);

        // init planners with parameters from config
        f1tenth::FrenetParams fp;
        fp.maxt = get_parameter("frenet_time_horizon").as_double();
        fp.mint = get_parameter("frenet_min_time").as_double();
        fp.target_speed = get_parameter("frenet_target_speed").as_double();
        fp.dt = get_parameter("frenet_dt").as_double();
        fp.max_speed = get_parameter("frenet_max_speed").as_double();
        fp.max_accel = get_parameter("frenet_max_accel").as_double();
        fp.d_samples = get_parameter("frenet_d_samples").as_double_array();
        fp.t_samples = get_parameter("frenet_t_samples").as_double_array();
        fp.k_j = get_parameter("frenet_k_jerk").as_double();
        fp.k_t = get_parameter("frenet_k_time").as_double();
        fp.k_d = get_parameter("frenet_k_deviation").as_double();
        fp.k_v = get_parameter("frenet_k_velocity").as_double();
        fp.safety_radius = get_parameter("frenet_safety_radius").as_double();
        fp.road_half_width = get_parameter("frenet_road_half_width").as_double();
        fp.log_level = static_cast<f1tenth::LogLevel>(get_parameter("log_level").as_int());

        // Enhanced safety parameters
        fp.vehicle_radius = get_parameter("frenet_vehicle_radius").as_double();
        fp.obstacle_radius = get_parameter("frenet_obstacle_radius").as_double();
        fp.k_velocity_safety = get_parameter("frenet_k_velocity_safety").as_double();
        fp.min_safety_margin = get_parameter("frenet_min_safety_margin").as_double();
        fp.k_proximity = get_parameter("frenet_k_proximity").as_double();
        fp.proximity_threshold = get_parameter("frenet_proximity_threshold").as_double();
        fp.interpolation_checks = get_parameter("frenet_interpolation_checks").as_int();

        frenet_ = std::make_unique<f1tenth::FrenetPlanner>(fp);


        f1tenth::LatticeConfig lc;
        lc.horizon = get_parameter("planner_horizon").as_double();
        lattice_ = std::make_unique<f1tenth::LatticeLUT>(lc);
        // lattice_ will be built after CSV path is loaded

        // Load CSV file if specified
        std::string csv_file = get_parameter("csv_file_path").as_string();
        if (!csv_file.empty() && !loadPathFromCSV(csv_file)) {
            RCLCPP_ERROR(get_logger(), "Failed to load CSV file: %s", csv_file.c_str());
        }


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer for publishing global path
        double publish_rate = get_parameter("publish_rate").as_double();
        global_path_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&PathPlannerNode::publishGlobalPath, this)
        );

        RCLCPP_INFO(get_logger(), "Path Planner Node initialized");
        RCLCPP_INFO(get_logger(), "Use Frenet: %s, Use Lattice: %s",
            get_parameter("use_frenet").as_bool() ? "true" : "false",
            get_parameter("use_lattice").as_bool() ? "true" : "false");
        if (!csv_file.empty()) {
            RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from CSV: %s",
                ref_wps_.size(), csv_file.c_str());
            if (!ref_wps_.empty()) {
                RCLCPP_INFO(get_logger(), "Path range: x=[%.2f, %.2f], y=[%.2f, %.2f], total length=%.2f m",
                    ref_wps_.front().x, ref_wps_.back().x,
                    ref_wps_.front().y, ref_wps_.back().y,
                    ref_wps_.back().s);
            }
        } else {
            RCLCPP_INFO(get_logger(), "No CSV file specified, waiting for global_path_topic");
        }
    }


private:
    // Obstacle bounding box structure
    struct ObstacleBox {
        double center_x, center_y;  // Center position
        double size;                 // Box size (square)
        std::vector<std::pair<double, double>> points;  // Original scan points in this cluster
    };

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = *msg;
        have_scan_ = true;
    }

    // Cluster scan points into obstacle groups using distance-based clustering
    std::vector<std::vector<std::pair<double, double>>> clusterObstacles(
        const std::vector<std::pair<double, double>>& points,
        double cluster_distance = 0.5)  // Points within 0.5m are in same cluster
    {
        std::vector<std::vector<std::pair<double, double>>> clusters;
        std::vector<bool> visited(points.size(), false);

        for (size_t i = 0; i < points.size(); ++i) {
            if (visited[i]) continue;

            std::vector<std::pair<double, double>> cluster;
            std::vector<size_t> to_check;
            to_check.push_back(i);

            while (!to_check.empty()) {
                size_t current = to_check.back();
                to_check.pop_back();

                if (visited[current]) continue;
                visited[current] = true;
                cluster.push_back(points[current]);

                // Find nearby points
                for (size_t j = 0; j < points.size(); ++j) {
                    if (visited[j]) continue;

                    double dx = points[current].first - points[j].first;
                    double dy = points[current].second - points[j].second;
                    double dist = std::sqrt(dx * dx + dy * dy);

                    if (dist < cluster_distance) {
                        to_check.push_back(j);
                    }
                }
            }

            // Only keep clusters with multiple points (filter noise)
            if (cluster.size() >= 3) {
                clusters.push_back(cluster);
            }
        }

        return clusters;
    }

    // Convert clusters to bounding boxes
    std::vector<ObstacleBox> createBoundingBoxes(
        const std::vector<std::vector<std::pair<double, double>>>& clusters)
    {
        std::vector<ObstacleBox> boxes;

        // Get parameters
        double max_box_size = get_parameter("obstacle_max_box_size").as_double();
        double safety_margin = get_parameter("obstacle_box_safety_margin").as_double();

        for (const auto& cluster : clusters) {
            if (cluster.empty()) continue;

            // Find min/max extents
            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();

            for (const auto& point : cluster) {
                min_x = std::min(min_x, point.first);
                max_x = std::max(max_x, point.first);
                min_y = std::min(min_y, point.second);
                max_y = std::max(max_y, point.second);
            }

            // Create square bounding box (use larger dimension)
            double width = max_x - min_x;
            double height = max_y - min_y;
            double size = std::max(width, height);

            // FILTER: Ignore boxes larger than max_box_size (likely walls or large structures)
            if (size > max_box_size) {
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Ignoring large obstacle box: size=%.2fm (exceeds max=%.2fm)", size, max_box_size);
                continue;
            }

            // Add safety margin
            size += safety_margin;

            ObstacleBox box;
            box.center_x = (min_x + max_x) / 2.0;
            box.center_y = (min_y + max_y) / 2.0;
            box.size = size;
            box.points = cluster;

            boxes.push_back(box);
        }

        return boxes;
    }

    // Visualize obstacle bounding boxes in RViz
    void visualizeObstacles(const std::vector<ObstacleBox>& obstacles)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Clear previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = get_parameter("frame_id").as_string();
        clear_marker.header.stamp = now();
        clear_marker.ns = "obstacle_boxes";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // Create markers for each obstacle
        for (size_t i = 0; i < obstacles.size(); ++i) {
            const auto& obs = obstacles[i];

            // Bounding box marker (CUBE)
            visualization_msgs::msg::Marker box_marker;
            box_marker.header.frame_id = get_parameter("frame_id").as_string();
            box_marker.header.stamp = now();
            box_marker.ns = "obstacle_boxes";
            box_marker.id = i * 2;  // Even IDs for boxes
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.action = visualization_msgs::msg::Marker::ADD;

            box_marker.pose.position.x = obs.center_x;
            box_marker.pose.position.y = obs.center_y;
            box_marker.pose.position.z = 0.5;  // Elevate for visibility
            box_marker.pose.orientation.w = 1.0;

            box_marker.scale.x = obs.size;
            box_marker.scale.y = obs.size;
            box_marker.scale.z = 1.0;  // 1m height

            // Red semi-transparent box
            box_marker.color.r = 1.0;
            box_marker.color.g = 0.0;
            box_marker.color.b = 0.0;
            box_marker.color.a = 0.4;

            marker_array.markers.push_back(box_marker);

            // Boundary outline (LINE_STRIP)
            visualization_msgs::msg::Marker outline_marker;
            outline_marker.header.frame_id = get_parameter("frame_id").as_string();
            outline_marker.header.stamp = now();
            outline_marker.ns = "obstacle_boxes";
            outline_marker.id = i * 2 + 1;  // Odd IDs for outlines
            outline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            outline_marker.action = visualization_msgs::msg::Marker::ADD;

            outline_marker.scale.x = 0.05;  // Line width
            outline_marker.pose.orientation.w = 1.0;

            // Bright red outline
            outline_marker.color.r = 1.0;
            outline_marker.color.g = 0.0;
            outline_marker.color.b = 0.0;
            outline_marker.color.a = 1.0;

            // Create square outline points
            double half_size = obs.size / 2.0;
            geometry_msgs::msg::Point p1, p2, p3, p4, p5;
            p1.x = obs.center_x - half_size; p1.y = obs.center_y - half_size; p1.z = 0.0;
            p2.x = obs.center_x + half_size; p2.y = obs.center_y - half_size; p2.z = 0.0;
            p3.x = obs.center_x + half_size; p3.y = obs.center_y + half_size; p3.z = 0.0;
            p4.x = obs.center_x - half_size; p4.y = obs.center_y + half_size; p4.z = 0.0;
            p5 = p1;  // Close the loop

            outline_marker.points.push_back(p1);
            outline_marker.points.push_back(p2);
            outline_marker.points.push_back(p3);
            outline_marker.points.push_back(p4);
            outline_marker.points.push_back(p5);

            marker_array.markers.push_back(outline_marker);
        }

        pub_obstacle_markers_->publish(marker_array);

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "Visualized %zu obstacle bounding boxes", obstacles.size());
    }

    // Convert laser scan to obstacles in map frame
    std::vector<std::pair<double, double>> getObstaclesFromScan()
    {
        std::vector<std::pair<double, double>> obstacles;

        if (!have_scan_) return obstacles;

        // Get transform from laser frame to map frame using TF
        geometry_msgs::msg::TransformStamped transform_stamped;
        std::string map_frame = get_parameter("frame_id").as_string();
        std::string laser_frame = latest_scan_.header.frame_id;

        try {
            // Look up the transform from laser frame to map frame
            // Use the timestamp from the scan for accurate transform
            transform_stamped = tf_buffer_->lookupTransform(
                map_frame, laser_frame,
                latest_scan_.header.stamp,
                rclcpp::Duration::from_seconds(0.1));

            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                "Transform lookup successful: %s → %s", laser_frame.c_str(), map_frame.c_str());
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Could not transform %s to %s: %s", laser_frame.c_str(), map_frame.c_str(), ex.what());
            return obstacles;
        }

        // Extract transform parameters
        double tx = transform_stamped.transform.translation.x;
        double ty = transform_stamped.transform.translation.y;

        // Get yaw from quaternion
        double qw = transform_stamped.transform.rotation.w;
        double qx = transform_stamped.transform.rotation.x;
        double qy = transform_stamped.transform.rotation.y;
        double qz = transform_stamped.transform.rotation.z;
        double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        // Convert laser scan points to map frame using TF transform
        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
            double range = latest_scan_.ranges[i];

            // Skip invalid readings
            if (std::isnan(range) || std::isinf(range)) continue;
            if (range < latest_scan_.range_min || range > latest_scan_.range_max) continue;

            // IMPROVED: Consider obstacles within 8 meters for better planning horizon
            if (range > 8.0) continue;

            // Calculate angle of this laser beam
            double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;

            // Point in laser frame
            double laser_x = range * std::cos(angle);
            double laser_y = range * std::sin(angle);

            // Transform to map frame using TF
            double map_x = tx + laser_x * std::cos(yaw) - laser_y * std::sin(yaw);
            double map_y = ty + laser_x * std::sin(yaw) + laser_y * std::cos(yaw);

            obstacles.push_back({map_x, map_y});
        }

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
            "Transformed %zu scan points to map frame using TF", obstacles.size());

        return obstacles;
    }

    bool loadPathFromCSV(const std::string& filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            return false;
        }

        ref_path_.header.frame_id = get_parameter("frame_id").as_string();
        ref_path_.poses.clear();
        ref_wps_.clear();

        std::string line;
        bool header_skipped = false;

        while (std::getline(file, line)) {
            // Skip header line if it contains non-numeric data
            if (!header_skipped && (line.find("x") != std::string::npos || line.find("y") != std::string::npos)) {
                header_skipped = true;
                continue;
            }

            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> cells;

            while (std::getline(ss, cell, ',')) {
                cells.push_back(cell);
            }

            if (cells.size() >= 2) {
                try {
                    double x = std::stod(cells[0]);
                    double y = std::stod(cells[1]);
                    double v = (cells.size() >= 3) ? std::stod(cells[2]) : 1.0;
                    double kappa = (cells.size() >= 4) ? std::stod(cells[3]) : 0.0;

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = ref_path_.header.frame_id;
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = v;  // Store velocity in z component
                    pose.pose.orientation.w = 1.0;

                    ref_path_.poses.push_back(pose);
                    ref_wps_.push_back({x, y, 0.0, 0.0, v}); // yaw and s will be calculated, v from CSV
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "Failed to parse line: %s", line.c_str());
                }
            }
        }

        file.close();

        // Calculate orientations and cumulative arc-length (s)
        if (ref_path_.poses.size() >= 2) {
            double s_accumulator = 0.0;

            // First, calculate s for all points
            ref_wps_[0].s = 0.0;
            for (size_t i = 1; i < ref_wps_.size(); ++i) {
                double segment_length = std::hypot(
                    ref_wps_[i].x - ref_wps_[i-1].x,
                    ref_wps_[i].y - ref_wps_[i-1].y
                );
                s_accumulator += segment_length;
                ref_wps_[i].s = s_accumulator;
            }

            // Check if path is closed (first and last points are close)
            double first_last_dist = std::hypot(
                ref_wps_.front().x - ref_wps_.back().x,
                ref_wps_.front().y - ref_wps_.back().y
            );
            bool is_closed = (first_last_dist < 2.0);  // Consider closed if within 2 meters

            // Calculate orientations
            for (size_t i = 0; i < ref_path_.poses.size() - 1; ++i) {
                double dx = ref_path_.poses[i + 1].pose.position.x - ref_path_.poses[i].pose.position.x;
                double dy = ref_path_.poses[i + 1].pose.position.y - ref_path_.poses[i].pose.position.y;
                double yaw = std::atan2(dy, dx);

                ref_path_.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
                ref_path_.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
                ref_wps_[i].yaw = yaw;
            }

            // Handle last waypoint orientation
            if (is_closed) {
                // For closed loop, calculate orientation from last to first
                double dx = ref_path_.poses[0].pose.position.x - ref_path_.poses.back().pose.position.x;
                double dy = ref_path_.poses[0].pose.position.y - ref_path_.poses.back().pose.position.y;
                double yaw = std::atan2(dy, dx);

                ref_path_.poses.back().pose.orientation.z = std::sin(yaw / 2.0);
                ref_path_.poses.back().pose.orientation.w = std::cos(yaw / 2.0);
                ref_wps_.back().yaw = yaw;

                RCLCPP_INFO(get_logger(), "Detected closed-loop track (gap: %.2f m)", first_last_dist);
            } else {
                // For open paths, use same orientation as second-to-last
                ref_path_.poses.back().pose.orientation = ref_path_.poses[ref_path_.poses.size() - 2].pose.orientation;
                ref_wps_.back().yaw = ref_wps_[ref_wps_.size() - 2].yaw;

                RCLCPP_INFO(get_logger(), "Detected open path (gap: %.2f m)", first_last_dist);
            }
        }

        if(frenet_ && !ref_wps_.empty()) {
            frenet_->set_reference(ref_wps_);

            // Build lattice LUT with Frenet frame reference
            if(lattice_) {
                RCLCPP_INFO(get_logger(), "Building Lattice LUT with Frenet frame...");
                lattice_->build(*frenet_);
                RCLCPP_INFO(get_logger(), "Lattice LUT built successfully");
            }
        }

        return !ref_wps_.empty();
    }

    void publishGlobalPath()
    {
        if (!ref_path_.poses.empty()) {
            ref_path_.header.stamp = now();
            for (auto& pose : ref_path_.poses) {
                pose.header.stamp = ref_path_.header.stamp;
            }
            pub_global_path_->publish(ref_path_);

            // Visualize velocity with color-coded markers
            visualizeGlobalPathVelocity();
        }
    }

    void visualizeGlobalPathVelocity()
    {
        if (ref_path_.poses.empty()) return;

        visualization_msgs::msg::MarkerArray marker_array;

        // Clear previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = get_parameter("frame_id").as_string();
        clear_marker.header.stamp = now();
        clear_marker.ns = "global_path_velocity";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // Find min and max velocity for color mapping
        double min_vel = std::numeric_limits<double>::max();
        double max_vel = std::numeric_limits<double>::min();
        for (const auto& pose : ref_path_.poses) {
            double vel = pose.pose.position.z;  // velocity stored in z
            min_vel = std::min(min_vel, vel);
            max_vel = std::max(max_vel, vel);
        }

        // Create line segments with color gradient
        for (size_t i = 0; i < ref_path_.poses.size() - 1; ++i) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = get_parameter("frame_id").as_string();
            line_marker.header.stamp = now();
            line_marker.ns = "global_path_velocity";
            line_marker.id = i;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.15;  // Line width
            line_marker.pose.orientation.w = 1.0;

            // Get velocities and normalize
            double vel1 = ref_path_.poses[i].pose.position.z;
            double vel2 = ref_path_.poses[i + 1].pose.position.z;
            double avg_vel = (vel1 + vel2) / 2.0;

            // Normalize velocity to [0, 1]
            double vel_ratio = (max_vel > min_vel) ?
                (avg_vel - min_vel) / (max_vel - min_vel) : 0.5;

            // Color gradient: Blue (slow) -> Green (medium) -> Red (fast)
            if (vel_ratio < 0.5) {
                // Blue to Green
                line_marker.color.r = 0.0;
                line_marker.color.g = vel_ratio * 2.0;
                line_marker.color.b = 1.0 - vel_ratio * 2.0;
            } else {
                // Green to Red
                line_marker.color.r = (vel_ratio - 0.5) * 2.0;
                line_marker.color.g = 1.0 - (vel_ratio - 0.5) * 2.0;
                line_marker.color.b = 0.0;
            }
            line_marker.color.a = 0.8;

            // Add two points for the line segment (at track level, z=0)
            geometry_msgs::msg::Point p1, p2;
            p1.x = ref_path_.poses[i].pose.position.x;
            p1.y = ref_path_.poses[i].pose.position.y;
            p1.z = 0.0;  // Track level
            p2.x = ref_path_.poses[i + 1].pose.position.x;
            p2.y = ref_path_.poses[i + 1].pose.position.y;
            p2.z = 0.0;  // Track level

            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);

            marker_array.markers.push_back(line_marker);
        }

        pub_velocity_markers_->publish(marker_array);
    }


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
        odom_received_ = true;

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
            "Received odometry: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f",
            odom_.pose.pose.position.x, odom_.pose.pose.position.y,
            odom_.twist.twist.linear.x, odom_.twist.twist.linear.y);

        // Execute path planning on every odometry update
        planPath();
    }


    void planPath()
    {
        // Start timing the entire planning process
        auto start_time = std::chrono::steady_clock::now();

        if(ref_wps_.size()<5) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Insufficient waypoints: %zu (need at least 5)", ref_wps_.size());
            return;
        }

        if(!odom_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "No odometry received yet");
            return;
        }

        // Calculate message delay (time since odometry was stamped)
        auto current_ros_time = now();
        auto odom_stamp = rclcpp::Time(odom_.header.stamp);
        double message_delay = (current_ros_time - odom_stamp).seconds();

        // Get current velocity and heading
        double vx = odom_.twist.twist.linear.x;
        double vy = odom_.twist.twist.linear.y;
        double v = std::hypot(vx, vy);

        // Extract yaw from quaternion
        double qx = odom_.pose.pose.orientation.x;
        double qy = odom_.pose.pose.orientation.y;
        double qz = odom_.pose.pose.orientation.z;
        double qw = odom_.pose.pose.orientation.w;
        double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz));

        // Estimate total delay (message delay + expected computation time)
        double expected_computation_time = get_parameter("expected_computation_time").as_double();
        double total_lookahead_time = message_delay + expected_computation_time;

        // Compensate position based on velocity and heading
        double x_current = odom_.pose.pose.position.x;
        double y_current = odom_.pose.pose.position.y;

        // Project position forward based on velocity and total delay
        double x_compensated = x_current + v * std::cos(yaw) * total_lookahead_time;
        double y_compensated = y_current + v * std::sin(yaw) * total_lookahead_time;

        // Log timing and compensation info
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Position compensation: delay=%.3fs (msg=%.3fs, comp=%.3fs), v=%.2fm/s, "
            "pos=(%.2f,%.2f)→(%.2f,%.2f), Δ=%.3fm",
            total_lookahead_time, message_delay, expected_computation_time, v,
            x_current, y_current, x_compensated, y_compensated,
            std::hypot(x_compensated - x_current, y_compensated - y_current));

        // Check if position is (0,0) which likely means uninitialized
        if (std::abs(x_current) < 1e-6 && std::abs(y_current) < 1e-6) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Position is (0,0), likely uninitialized odometry. Waiting for valid position.");
            return;
        }

        // Use compensated position for planning
        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom_.header;
        pose.pose.position.x = x_compensated;
        pose.pose.position.y = y_compensated;
        pose.pose.position.z = odom_.pose.pose.position.z;
        pose.pose.orientation = odom_.pose.pose.orientation;

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "Compensated pose: x=%.2f, y=%.2f (original: x=%.2f, y=%.2f)",
            x_compensated, y_compensated, x_current, y_current);

        // Convert compensated position to Frenet frame
        auto frenet_start = std::chrono::steady_clock::now();
        size_t idx; f1tenth::FrenetState fs;
        if(!frenet_->cart2frenet(x_compensated, y_compensated, idx, fs)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Failed to convert to Frenet frame at compensated position x=%.2f, y=%.2f",
                x_compensated, y_compensated);
            return;
        }
        fs.ds = v;  // Use actual speed

        // IMPROVED: Add lookahead distance to start planning ahead of current position
        double lookahead_distance = get_parameter("frenet_lookahead_distance").as_double();
        double original_s = fs.s;
        fs.s += lookahead_distance;

        // Handle wrapping for closed loops
        if (frenet_->ref_size() > 0) {
            // Get total path length from the last waypoint's s-coordinate
            double total_length = ref_wps_.back().s;

            // Check if path is closed (first and last waypoints are close)
            double first_last_dist = std::hypot(
                ref_wps_.front().x - ref_wps_.back().x,
                ref_wps_.front().y - ref_wps_.back().y
            );
            bool is_closed = (first_last_dist < 2.0);

            if (is_closed) {
                total_length += first_last_dist;
                // Wrap s if it exceeds total length
                if (fs.s >= total_length) {
                    fs.s = std::fmod(fs.s, total_length);
                }
            }
        }

        auto frenet_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - frenet_start).count();

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "Lookahead applied: original_s=%.2f, lookahead=%.2f, new_s=%.2f",
            original_s, lookahead_distance, fs.s);

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "Frenet state: s=%.2f, d=%.2f, ds=%.2f (conversion time: %.3fms)",
            fs.s, fs.d, fs.ds, frenet_time * 1000.0);

        // Get obstacles from laser scan and model as bounding boxes
        auto obstacle_start = std::chrono::steady_clock::now();

        // Step 1: Get raw scan points in map frame
        std::vector<std::pair<double,double>> raw_obstacles = getObstaclesFromScan();

        // Step 2: Cluster scan points into obstacle groups
        double cluster_distance = get_parameter("obstacle_cluster_distance").as_double();
        auto clusters = clusterObstacles(raw_obstacles, cluster_distance);

        // Step 3: Create bounding boxes for each cluster
        auto obstacle_boxes = createBoundingBoxes(clusters);

        // Step 4: Visualize bounding boxes in RViz
        visualizeObstacles(obstacle_boxes);

        // Step 5: Convert bounding boxes to obstacle points for Frenet planner
        // Sample points around the perimeter of each bounding box
        std::vector<std::pair<double,double>> obstacles;
        for (const auto& box : obstacle_boxes) {
            double half_size = box.size / 2.0;
            int points_per_side = 8;  // Sample 8 points per side for dense coverage

            // Sample all 4 sides of the square
            for (int i = 0; i < points_per_side; ++i) {
                double t = static_cast<double>(i) / (points_per_side - 1);

                // Bottom side
                obstacles.push_back({
                    box.center_x - half_size + t * box.size,
                    box.center_y - half_size
                });

                // Top side
                obstacles.push_back({
                    box.center_x - half_size + t * box.size,
                    box.center_y + half_size
                });

                // Left side
                obstacles.push_back({
                    box.center_x - half_size,
                    box.center_y - half_size + t * box.size
                });

                // Right side
                obstacles.push_back({
                    box.center_x + half_size,
                    box.center_y - half_size + t * box.size
                });
            }
        }

        auto obstacle_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - obstacle_start).count();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Obstacles: %zu raw pts → %zu clusters → %zu boxes (filtered) → %zu boundary pts | Time: %.1fms",
            raw_obstacles.size(), clusters.size(), obstacle_boxes.size(), obstacles.size(), obstacle_time * 1000.0);

        // DEBUG: Log obstacle details
        if (!obstacles.empty()) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                "First obstacle point: (%.2f, %.2f), Total obstacles: %zu",
                obstacles[0].first, obstacles[0].second, obstacles.size());
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "WARNING: No obstacles to pass to Frenet planner!");
        }

        // Initialize best segment container
        std::vector<f1tenth::Waypoint> best_seg;

        // Generate frenet candidates if enabled
        std::optional<f1tenth::FrenetTraj> best_frenet;

        if (get_parameter("use_frenet").as_bool()) {
            auto frenet_gen_start = std::chrono::steady_clock::now();

            // DEBUG: Log number of obstacles being passed to Frenet planner
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                "Passing %zu obstacle points to Frenet planner", obstacles.size());

            // Generate multiple trajectory candidates using Frenet lattice
            auto cands = frenet_->generate(fs, obstacles);
            auto frenet_gen_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - frenet_gen_start).count();
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                "Generated %zu Frenet trajectory candidates (time: %.3fms)", cands.size(), frenet_gen_time * 1000.0);

            // Select best trajectory (considers obstacles and cost)
            auto select_start = std::chrono::steady_clock::now();
            best_frenet = frenet_->select_best(cands);
            auto select_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - select_start).count();

            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                "Selected best Frenet trajectory (time: %.3fms)", select_time * 1000.0);

            if(!best_frenet) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "No valid Frenet trajectory found, using fallback");
                // Fallback: Follow centerline at reduced speed
                f1tenth::FrenetTraj fallback;
                for(double t = 0; t <= 2.0; t += 0.1) {
                    double s = fs.s + fs.ds * t;
                    double d = fs.d * std::exp(-t); // Converge to centerline
                    double x_f, y_f, yaw_f;
                    if(frenet_->frenet2cart(s, d, x_f, y_f, yaw_f)) {
                        fallback.x.push_back(x_f);
                        fallback.y.push_back(y_f);
                        fallback.yaw.push_back(yaw_f);
                        fallback.s.push_back(s);
                        fallback.d.push_back(d);
                    }
                }
                if(!fallback.x.empty()) {
                    best_frenet = fallback;
                }
            }
        } else {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                "Frenet planner disabled, using reference path directly");
            // Use reference path directly without Frenet optimization
            size_t start_idx = idx;
            size_t end_idx = std::min(start_idx + 20, ref_wps_.size());
            for(size_t i = start_idx; i < end_idx; ++i) {
                best_seg.push_back(ref_wps_[i]);
            }
        }

        // Visualize Frenet path if enabled
        if (get_parameter("visualize_paths").as_bool() && best_frenet) {
            visualizeFrenetPath(*best_frenet);
            visualizeFrenetPathVelocity(*best_frenet);  // Color-coded velocity visualization
        }


        // Lattice goal sampling at horizon ahead along centerline
        nav_msgs::msg::Path out; out.header.frame_id = get_parameter("frame_id").as_string();


        if(best_frenet){
            // Use the full frenet path (don't skip points to ensure sufficient path length)
            for(size_t i=0;i<best_frenet->x.size();++i){
                double velocity = (i < best_frenet->v.size()) ? best_frenet->v[i] : 0.0;
                best_seg.push_back({best_frenet->x[i], best_frenet->y[i], best_frenet->yaw[i], 0.0, velocity});
            }
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                "Frenet path segment size: %zu (%.2f seconds coverage)",
                best_seg.size(), best_frenet->t.empty() ? 0.0 : best_frenet->t.back());
        }

        // augment with lattice (goal alignment) if enabled
        if(!best_seg.empty() && get_parameter("use_lattice").as_bool()){
            auto lattice_start = std::chrono::steady_clock::now();

            // Generate and visualize all lattice candidates from current position
            std::vector<std::vector<f1tenth::Waypoint>> all_lattice_paths;

            // compute relative goal at horizon
            const auto &p0 = best_seg.front();
            const auto &pH = best_seg.back();
            double dx = pH.x - p0.x;
            double dy = pH.y - p0.y;
            double dth = f1tenth::wrapAngle(pH.yaw - p0.yaw);

            // Generate multiple lattice candidates for visualization
            const int n_samples = 5;  // Number of lateral samples
            const double d_range = 1.0;  // Lateral range in meters

            for(int i = -n_samples/2; i <= n_samples/2; ++i) {
                double d_offset = i * (d_range / n_samples);

                // Query lattice with lateral offset
                auto sp = lattice_->query(dx, dy + d_offset, dth);
                if(sp){
                    // Sample lattice path from current position using Frenet frame
                    auto lut_seg = lattice_->sample(*sp, 0.05, *frenet_, fs.s, fs.d);
                    if(!lut_seg.empty()) {
                        all_lattice_paths.push_back(lut_seg);

                        // Use center path as the best segment
                        if(i == 0) {
                            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                "Lattice query successful: dx=%.2f, dy=%.2f, dth=%.2f", dx, dy, dth);
                            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                "Lattice sampled %zu waypoints from current position (s=%.2f, d=%.2f)",
                                lut_seg.size(), fs.s, fs.d);

                            // Add to best segment
                            for(const auto &w: lut_seg){
                                best_seg.push_back(w);
                            }
                        }
                    }
                }
            }

            // Visualize all lattice path candidates if enabled
            if (get_parameter("visualize_paths").as_bool() && !all_lattice_paths.empty()) {
                visualizeAllLatticePaths(all_lattice_paths);
            }

            auto lattice_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - lattice_start).count();
            RCLCPP_DEBUG(get_logger(), "Lattice planning time: %.3fms for %zu candidates",
                         lattice_time * 1000.0, all_lattice_paths.size());
        } else if (!best_seg.empty() && !get_parameter("use_lattice").as_bool()) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                "Lattice planner disabled");
        }
        // onTimer()

        // Publish
        if(!best_seg.empty()){
            nav_msgs::msg::Path p = f1tenth::waypointsToPathMsg(best_seg, out.header.frame_id);
            p.header.stamp = now();
            pub_path_->publish(p);

            // Calculate total planning time
            auto total_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "Published planned_path with %zu waypoints | Total planning time: %.3fms "
                "(actual computation: %.3fms vs expected: %.3fms)",
                p.poses.size(), total_time * 1000.0,
                total_time * 1000.0, expected_computation_time * 1000.0);

            // Update expected computation time with exponential moving average if enabled
            if (get_parameter("adaptive_compensation").as_bool()) {
                double alpha = 0.2;  // Smoothing factor
                double new_expected = alpha * total_time + (1.0 - alpha) * expected_computation_time;
                set_parameter(rclcpp::Parameter("expected_computation_time", new_expected));

                RCLCPP_DEBUG(get_logger(), "Updated expected computation time: %.3fms → %.3fms",
                             expected_computation_time * 1000.0, new_expected * 1000.0);
            }
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "No path to publish (best_seg is empty)");
        }
    }


    // Helper function: Get velocity from global path at given s coordinate
    double getVelocityAtS(double s_query) const
    {
        if (ref_path_.poses.empty()) {
            return 0.0;
        }

        // Calculate accumulated s for each waypoint in ref_path_
        std::vector<double> s_values;
        s_values.push_back(0.0);
        double accumulated_s = 0.0;

        for (size_t i = 1; i < ref_path_.poses.size(); ++i) {
            double dx = ref_path_.poses[i].pose.position.x - ref_path_.poses[i-1].pose.position.x;
            double dy = ref_path_.poses[i].pose.position.y - ref_path_.poses[i-1].pose.position.y;
            accumulated_s += std::hypot(dx, dy);
            s_values.push_back(accumulated_s);
        }

        double total_length = accumulated_s;

        // Handle closed loop (wrap s coordinate)
        if (ref_path_.poses.size() >= 2) {
            double dx = ref_path_.poses[0].pose.position.x - ref_path_.poses.back().pose.position.x;
            double dy = ref_path_.poses[0].pose.position.y - ref_path_.poses.back().pose.position.y;
            double closing_distance = std::hypot(dx, dy);

            if (closing_distance < 2.0) {
                // Closed loop detected
                total_length += closing_distance;

                // Wrap s_query to [0, total_length)
                while (s_query < 0) s_query += total_length;
                while (s_query >= total_length) s_query -= total_length;
            }
        }

        // Find the two closest waypoints for interpolation
        size_t lower_idx = 0;
        for (size_t i = 0; i < s_values.size(); ++i) {
            if (s_values[i] <= s_query) {
                lower_idx = i;
            } else {
                break;
            }
        }

        // Handle edge cases
        if (lower_idx >= ref_path_.poses.size() - 1) {
            // At or beyond last waypoint
            return ref_path_.poses.back().pose.position.z;
        }

        // Linear interpolation
        size_t upper_idx = lower_idx + 1;
        double s_lower = s_values[lower_idx];
        double s_upper = s_values[upper_idx];
        double v_lower = ref_path_.poses[lower_idx].pose.position.z;
        double v_upper = ref_path_.poses[upper_idx].pose.position.z;

        if (std::abs(s_upper - s_lower) < 1e-6) {
            return v_lower;
        }

        double t = (s_query - s_lower) / (s_upper - s_lower);
        t = std::max(0.0, std::min(1.0, t));  // Clamp to [0, 1]

        return v_lower + t * (v_upper - v_lower);
    }

    void visualizeFrenetPath(const f1tenth::FrenetTraj& path)
    {
        nav_msgs::msg::Path frenet_msg;
        frenet_msg.header.frame_id = get_parameter("frame_id").as_string();
        frenet_msg.header.stamp = now();

        for (size_t i = 0; i < path.x.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = frenet_msg.header;
            pose.pose.position.x = path.x[i];
            pose.pose.position.y = path.y[i];

            // Get velocity from global path at the same s coordinate
            double s_coord = (i < path.s.size()) ? path.s[i] : 0.0;
            double velocity = getVelocityAtS(s_coord);

            // Store velocity in z component for path_tracker to use
            pose.pose.position.z = velocity;

            double yaw = path.yaw[i];
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);

            frenet_msg.poses.push_back(pose);
        }

        pub_frenet_path_->publish(frenet_msg);
    }

    void visualizeFrenetPathVelocity(const f1tenth::FrenetTraj& path)
    {
        if (path.x.size() < 2) return;

        visualization_msgs::msg::MarkerArray marker_array;

        // Clear previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = get_parameter("frame_id").as_string();
        clear_marker.header.stamp = now();
        clear_marker.ns = "frenet_path_velocity";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // Get velocities from global path at corresponding s coordinates
        std::vector<double> velocities;
        for (size_t i = 0; i < path.x.size(); ++i) {
            double s_coord = (i < path.s.size()) ? path.s[i] : 0.0;
            double velocity = getVelocityAtS(s_coord);
            velocities.push_back(velocity);
        }

        // Find min and max velocity for color mapping
        double min_vel = *std::min_element(velocities.begin(), velocities.end());
        double max_vel = *std::max_element(velocities.begin(), velocities.end());

        // Create line segments with color gradient
        for (size_t i = 0; i < path.x.size() - 1; ++i) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = get_parameter("frame_id").as_string();
            line_marker.header.stamp = now();
            line_marker.ns = "frenet_path_velocity";
            line_marker.id = i + 1;  // Start from 1 (0 is DELETEALL)
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.25;  // IMPROVED: Thicker line width (25cm) for better visibility
            line_marker.pose.orientation.w = 1.0;

            // Average velocity of the segment
            double avg_vel = (velocities[i] + velocities[i + 1]) / 2.0;

            // Normalize velocity to [0, 1]
            double vel_ratio = (max_vel > min_vel) ?
                (avg_vel - min_vel) / (max_vel - min_vel) : 0.5;

            // Color gradient: Blue (slow) -> Green (medium) -> Red (fast)
            if (vel_ratio < 0.5) {
                // Blue to Green
                line_marker.color.r = 0.0;
                line_marker.color.g = vel_ratio * 2.0;
                line_marker.color.b = 1.0 - vel_ratio * 2.0;
            } else {
                // Green to Red
                line_marker.color.r = (vel_ratio - 0.5) * 2.0;
                line_marker.color.g = 1.0 - (vel_ratio - 0.5) * 2.0;
                line_marker.color.b = 0.0;
            }
            line_marker.color.a = 1.0;  // IMPROVED: Fully opaque for better visibility

            // Add two points for the line segment (elevated for better visibility)
            geometry_msgs::msg::Point p1, p2;
            p1.x = path.x[i];
            p1.y = path.y[i];
            p1.z = 0.15;  // IMPROVED: Higher elevation (15cm) for better visibility above ground
            p2.x = path.x[i + 1];
            p2.y = path.y[i + 1];
            p2.z = 0.15;  // IMPROVED: Higher elevation (15cm)

            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);

            marker_array.markers.push_back(line_marker);
        }

        // ADDED: Create a single bright marker for the entire path outline (easier to see)
        visualization_msgs::msg::Marker outline_marker;
        outline_marker.header.frame_id = get_parameter("frame_id").as_string();
        outline_marker.header.stamp = now();
        outline_marker.ns = "frenet_path_outline";
        outline_marker.id = 0;
        outline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        outline_marker.action = visualization_msgs::msg::Marker::ADD;
        outline_marker.scale.x = 0.3;  // Even thicker for outline (30cm)
        outline_marker.pose.orientation.w = 1.0;

        // Bright cyan color for high visibility
        outline_marker.color.r = 0.0;
        outline_marker.color.g = 1.0;
        outline_marker.color.b = 1.0;
        outline_marker.color.a = 0.7;  // Slightly transparent to not obscure velocity colors

        // Add all path points
        for (size_t i = 0; i < path.x.size(); ++i) {
            geometry_msgs::msg::Point p;
            p.x = path.x[i];
            p.y = path.y[i];
            p.z = 0.12;  // Slightly below velocity markers
            outline_marker.points.push_back(p);
        }
        marker_array.markers.push_back(outline_marker);

        pub_frenet_velocity_markers_->publish(marker_array);
    }

    void visualizeLUTPath(const std::vector<f1tenth::Waypoint>& path)
    {
        nav_msgs::msg::Path lut_msg;
        lut_msg.header.frame_id = get_parameter("frame_id").as_string();
        lut_msg.header.stamp = now();

        for (const auto& wp : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = lut_msg.header;
            pose.pose.position.x = wp.x;
            pose.pose.position.y = wp.y;

            // Store velocity in z component for path_tracker to use
            pose.pose.position.z = wp.v;

            pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
            pose.pose.orientation.w = std::cos(wp.yaw / 2.0);

            lut_msg.poses.push_back(pose);
        }

        pub_lut_path_->publish(lut_msg);
    }

    void visualizeAllLatticePaths(const std::vector<std::vector<f1tenth::Waypoint>>& all_paths)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Clear previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = get_parameter("frame_id").as_string();
        clear_marker.header.stamp = now();
        clear_marker.ns = "lattice_candidates";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // Add all path candidates as line strips
        for (size_t path_idx = 0; path_idx < all_paths.size(); ++path_idx) {
            const auto& path = all_paths[path_idx];
            if (path.empty()) continue;

            visualization_msgs::msg::Marker path_marker;
            path_marker.header.frame_id = get_parameter("frame_id").as_string();
            path_marker.header.stamp = now();
            path_marker.ns = "lattice_candidates";
            path_marker.id = path_idx;
            path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::msg::Marker::ADD;

            // Set scale and color based on path index
            path_marker.scale.x = 0.08;  // IMPROVED: Thicker line width (8cm) for better visibility

            // Color gradient from red (outer) to green (center) to blue (outer)
            double color_ratio = static_cast<double>(path_idx) / static_cast<double>(all_paths.size() - 1);
            if (path_idx == all_paths.size() / 2) {
                // Center path (best path) in bright green
                path_marker.color.r = 0.0;
                path_marker.color.g = 1.0;
                path_marker.color.b = 0.0;
                path_marker.color.a = 1.0;  // Fully opaque for best path
            } else {
                // Other paths in gradient colors
                path_marker.color.r = 1.0 - std::abs(color_ratio - 0.5) * 2.0;
                path_marker.color.g = std::abs(color_ratio - 0.5) * 2.0;
                path_marker.color.b = 0.5;
                path_marker.color.a = 0.5;  // IMPROVED: More transparent for candidate paths
            }

            // Add points to line strip
            for (const auto& wp : path) {
                geometry_msgs::msg::Point point;
                point.x = wp.x;
                point.y = wp.y;
                point.z = 0.2;  // Slightly above ground
                path_marker.points.push_back(point);
            }

            marker_array.markers.push_back(path_marker);
        }

        // Publish all lattice candidate paths
        pub_markers_->publish(marker_array);

        RCLCPP_DEBUG(get_logger(), "Visualized %zu lattice path candidates", all_paths.size());
    }

    // members
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_frenet_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_lut_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_velocity_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_frenet_velocity_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacle_markers_;


    nav_msgs::msg::Path ref_path_;
    std::vector<f1tenth::Waypoint> ref_wps_;
    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::LaserScan latest_scan_;
    bool odom_received_;
    bool have_scan_;


    std::unique_ptr<f1tenth::FrenetPlanner> frenet_;
    std::unique_ptr<f1tenth::LatticeLUT> lattice_;


    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr global_path_timer_;
};


int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}