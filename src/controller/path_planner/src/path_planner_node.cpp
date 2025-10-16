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
        declare_parameter<std::vector<double>>("frenet_d_samples", std::vector<double>{-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0});
        declare_parameter<std::vector<double>>("frenet_t_samples", std::vector<double>{1.5, 2.0, 2.5, 3.0});
        declare_parameter<double>("frenet_k_jerk", 0.1);
        declare_parameter<double>("frenet_k_time", 0.1);
        declare_parameter<double>("frenet_k_deviation", 1.0);
        declare_parameter<double>("frenet_k_velocity", 1.0);
        declare_parameter<double>("frenet_safety_radius", 0.3);
        declare_parameter<double>("frenet_road_half_width", 1.2);



        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(get_parameter("odom_topic").as_string(), 30,
        std::bind(&PathPlannerNode::odomCallback, this, _1));

        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(get_parameter("scan_topic").as_string(), 30,
        std::bind(&PathPlannerNode::scanCallback, this, _1));

        // Publishers
        pub_path_ = create_publisher<nav_msgs::msg::Path>(get_parameter("planned_path_topic").as_string(),10);
        pub_global_path_ = create_publisher<nav_msgs::msg::Path>(get_parameter("global_path_topic").as_string(),10);

        // Visualization publishers
        if (get_parameter("visualize_paths").as_bool()) {
            pub_frenet_path_ = create_publisher<nav_msgs::msg::Path>(get_parameter("frenet_path_topic").as_string(),10);
            pub_lut_path_ = create_publisher<nav_msgs::msg::Path>(get_parameter("lut_path_topic").as_string(),10);
            pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/path_planner_markers",10);
        }

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
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = *msg;
        have_scan_ = true;
    }

    // Convert laser scan to obstacles in map frame
    std::vector<std::pair<double, double>> getObstaclesFromScan()
    {
        std::vector<std::pair<double, double>> obstacles;

        if (!have_scan_) return obstacles;

        // Get robot pose from odometry
        double robot_x = odom_.pose.pose.position.x;
        double robot_y = odom_.pose.pose.position.y;

        // Get robot yaw from quaternion
        double qw = odom_.pose.pose.orientation.w;
        double qx = odom_.pose.pose.orientation.x;
        double qy = odom_.pose.pose.orientation.y;
        double qz = odom_.pose.pose.orientation.z;
        double robot_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        // Convert laser scan points to map frame
        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
            double range = latest_scan_.ranges[i];

            // Skip invalid readings
            if (std::isnan(range) || std::isinf(range)) continue;
            if (range < latest_scan_.range_min || range > latest_scan_.range_max) continue;

            // Skip far obstacles (only consider nearby obstacles)
            if (range > 5.0) continue;

            // Calculate angle of this laser beam
            double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;

            // Convert to map frame
            double local_x = range * std::cos(angle);
            double local_y = range * std::sin(angle);

            // Rotate by robot yaw and translate to robot position
            double map_x = robot_x + local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw);
            double map_y = robot_y + local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw);

            obstacles.push_back({map_x, map_y});
        }

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
        }
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
        auto frenet_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - frenet_start).count();

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "Frenet state: s=%.2f, d=%.2f, ds=%.2f (conversion time: %.3fms)",
            fs.s, fs.d, fs.ds, frenet_time * 1000.0);

        // Get obstacles from laser scan
        auto obstacle_start = std::chrono::steady_clock::now();
        std::vector<std::pair<double,double>> obstacles = getObstaclesFromScan();
        auto obstacle_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - obstacle_start).count();
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "Detected %zu obstacles from laser scan (time: %.3fms)", obstacles.size(), obstacle_time * 1000.0);

        // Initialize best segment container
        std::vector<f1tenth::Waypoint> best_seg;

        // Generate frenet candidates if enabled
        std::optional<f1tenth::FrenetTraj> best_frenet;

        if (get_parameter("use_frenet").as_bool()) {
            auto frenet_gen_start = std::chrono::steady_clock::now();
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
            pose.pose.position.z = 0.1; // Slightly elevated for visibility

            double yaw = path.yaw[i];
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);

            frenet_msg.poses.push_back(pose);
        }

        pub_frenet_path_->publish(frenet_msg);
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
            pose.pose.position.z = 0.15; // Different elevation than Frenet

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
            path_marker.scale.x = 0.05;  // Line width

            // Color gradient from red (outer) to green (center) to blue (outer)
            double color_ratio = static_cast<double>(path_idx) / static_cast<double>(all_paths.size() - 1);
            if (path_idx == all_paths.size() / 2) {
                // Center path (best path) in green
                path_marker.color.r = 0.0;
                path_marker.color.g = 1.0;
                path_marker.color.b = 0.0;
                path_marker.color.a = 1.0;
            } else {
                // Other paths in gradient colors
                path_marker.color.r = 1.0 - std::abs(color_ratio - 0.5) * 2.0;
                path_marker.color.g = std::abs(color_ratio - 0.5) * 2.0;
                path_marker.color.b = 0.5;
                path_marker.color.a = 0.7;
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