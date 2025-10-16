#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CSVPathPublisher : public rclcpp::Node
{
public:
    CSVPathPublisher() : Node("csv_path_publisher")
    {
        // Declare parameters
        declare_parameter<std::string>("csv_file_path", "");
        declare_parameter<std::string>("global_path_topic", "/global_centerline");
        declare_parameter<std::string>("frame_id", "map");
        declare_parameter<double>("publish_rate", 1.0);

        // Get parameters
        csv_file_path_ = get_parameter("csv_file_path").as_string();
        global_path_topic_ = get_parameter("global_path_topic").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        double publish_rate = get_parameter("publish_rate").as_double();

        // Validate CSV file path
        if (csv_file_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "csv_file_path parameter is required!");
            rclcpp::shutdown();
            return;
        }

        // Create publisher
        path_publisher_ = create_publisher<nav_msgs::msg::Path>(global_path_topic_, 10);

        // Load path from CSV
        if (!load_path_from_csv()) {
            RCLCPP_ERROR(get_logger(), "Failed to load path from CSV file: %s", csv_file_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Create timer for publishing
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&CSVPathPublisher::publish_path, this)
        );

        RCLCPP_INFO(get_logger(), "CSV Path Publisher initialized");
        RCLCPP_INFO(get_logger(), "CSV file: %s", csv_file_path_.c_str());
        RCLCPP_INFO(get_logger(), "Publishing on topic: %s", global_path_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints", global_path_.poses.size());
    }

private:
    bool load_path_from_csv()
    {
        std::ifstream file(csv_file_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Could not open CSV file: %s", csv_file_path_.c_str());
            return false;
        }

        global_path_.header.frame_id = frame_id_;
        global_path_.poses.clear();

        std::string line;
        bool header_skipped = false;

        while (std::getline(file, line)) {
            // Skip header line if it contains non-numeric data
            if (!header_skipped && (line.find("x") != std::string::npos || line.find("y") != std::string::npos)) {
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
                    double x = std::stod(cells[0]);
                    double y = std::stod(cells[1]);
                    double v = (cells.size() >= 3) ? std::stod(cells[2]) : 1.0;  // Parse velocity

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = frame_id_;
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = v;  // Store velocity in z component

                    // Set orientation (can be improved with heading calculation between waypoints)
                    pose.pose.orientation.x = 0.0;
                    pose.pose.orientation.y = 0.0;
                    pose.pose.orientation.z = 0.0;
                    pose.pose.orientation.w = 1.0;

                    global_path_.poses.push_back(pose);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "Failed to parse line: %s. Error: %s", line.c_str(), e.what());
                    continue;
                }
            }
        }

        file.close();

        if (global_path_.poses.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid waypoints found in CSV file");
            return false;
        }

        // Calculate orientations based on path direction
        calculate_path_orientations();

        return true;
    }

    void calculate_path_orientations()
    {
        if (global_path_.poses.size() < 2) {
            return;
        }

        // Calculate orientation for each pose based on direction to next waypoint
        for (size_t i = 0; i < global_path_.poses.size() - 1; ++i) {
            double dx = global_path_.poses[i + 1].pose.position.x - global_path_.poses[i].pose.position.x;
            double dy = global_path_.poses[i + 1].pose.position.y - global_path_.poses[i].pose.position.y;
            double yaw = std::atan2(dy, dx);

            // Convert yaw to quaternion
            global_path_.poses[i].pose.orientation.x = 0.0;
            global_path_.poses[i].pose.orientation.y = 0.0;
            global_path_.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
            global_path_.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
        }

        // Set last pose orientation same as second-to-last
        if (global_path_.poses.size() >= 2) {
            global_path_.poses.back().pose.orientation = global_path_.poses[global_path_.poses.size() - 2].pose.orientation;
        }
    }

    void publish_path()
    {
        global_path_.header.stamp = now();

        // Update timestamp for all poses
        for (auto& pose : global_path_.poses) {
            pose.header.stamp = global_path_.header.stamp;
        }

        path_publisher_->publish(global_path_);

        RCLCPP_DEBUG(get_logger(), "Published path with %zu waypoints", global_path_.poses.size());
    }

    // Member variables
    std::string csv_file_path_;
    std::string global_path_topic_;
    std::string frame_id_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path global_path_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CSVPathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}