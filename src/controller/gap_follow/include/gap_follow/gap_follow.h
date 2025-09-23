#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

#define _USR_MATH_DEFINES
#include <cmath>
#include <memory>

struct gap
{
    int start_index;
    int end_index;
    float range_sum;
    float max_range;
    float min_range;
    int length;
    int best_index = 0;
    gap(): start_index(0), end_index(0), range_sum(0.0), max_range(0.0), min_range(1e3), length(0), best_index(-1) {}
    gap(int a, int b): start_index(a), end_index(b), range_sum(0.0), max_range(0.0), min_range(1e3), length(0), best_index(-1) {}
    // gap(int a, int b, int c): start_index(a), end_index(b), best_index(c) {}
    void operator()(int a, int b) {
        start_index = a;
        end_index = b;
    }
};

enum gap_versions
{
    gap_wider,
    gap_deeper,
    gap_spacier,
    gap_combi
};

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap();
    virtual ~ReactiveFollowGap();

private:
    // ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr steer_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr best_gap_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr many_gap_pub_;

    // callbacks
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    std::string scan_topic = "/scan";
    std::string drive_topic = "/drive";

    std::string base_frame = "ego_racecar/base_link";
    std::string scan_frame = "ego_racecar/laser";

    const int scan_indices = 1080;

    float far_threshold, near_threshold, gap_threshold, vehicle_width, safety_gain, wheel_base, space_gain;
    float gap_leng_filter, gap_maxrange_filter;

    gap_versions gap_ver;
    int best_ver;

    float speed;

    void preprocess_lidar(float* ranges);
    void find_gap(float* ranges, std::vector<gap>& gap_array);

    void find_max_gap(float* ranges, gap& max_gap);
    void find_best_point(float* ranges, gap& best_gap);
    float speed_profile(float gap_width, float gap_depth);
    float steering_profile(float* ranges, int & index);

    // visualization
    void visualize_steering(const float steering);
    void conversion_coordinate(float* ranges, int & index, float & x, float & y);
    void conversion_cartesian(const float & r, const float & theta, float & x, float & y);
    void visualize_many_gap(float* ranges, std::vector<gap>& many_gap);
    void visualize_best_gap(float* ranges, gap& best_gap);
};