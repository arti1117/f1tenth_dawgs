#include "gap_follow/gap_follow.h"

ReactiveFollowGap::~ReactiveFollowGap() {}
ReactiveFollowGap::ReactiveFollowGap() : rclcpp::Node("reactive_node")
{
    /// TODO: create ROS subscribers and publishers
    ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));

    steer_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("steering_arrow", 10);
    best_gap_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("best_gap_marker", 10);
    many_gap_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("many_gap_marker", 10);

    this->declare_parameter<float>("far_threshold", 3.0);
    this->declare_parameter<float>("near_threshold", 0.15);
    this->declare_parameter<float>("gap_threshold", 0.08);
    this->declare_parameter<float>("vehicle_width", 0.3);
    this->declare_parameter<float>("safety_gain", 1.0);
    this->declare_parameter<float>("wheel_base", 0.33);
    this->declare_parameter<float>("space_gain", 0.6);
    this->declare_parameter<float>("gap_maxrange_filter", 1.1);

    this->declare_parameter<int>("gap_ver", 0);
    this->declare_parameter<int>("best_ver", 0);
    this->declare_parameter<int>("gap_leng_filter", 10);

    this->declare_parameter<float>("speed", 1);

    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("base_frame", "ego_racecar/base_link");
    this->declare_parameter<std::string>("scan_frame", "ego_racecar/laser");

    far_threshold = this->get_parameter("far_threshold").as_double();
    near_threshold = this->get_parameter("near_threshold").as_double();
    gap_threshold = this->get_parameter("gap_threshold").as_double();
    vehicle_width = this->get_parameter("vehicle_width").as_double();
    safety_gain = this->get_parameter("safety_gain").as_double();
    wheel_base = this->get_parameter("wheel_base").as_double();

    this->get_parameter<std::string>("scan_topic", scan_topic);
    this->get_parameter<std::string>("drive_topic", drive_topic);
    this->get_parameter<std::string>("base_frame", base_frame);
    this->get_parameter<std::string>("scan_frame", scan_frame);


}


void ReactiveFollowGap::preprocess_lidar(float* ranges)
{
    // Preprocess the LiDAR scan array. Expert implementation includes:
    // 1.Setting each value to the mean over some window
    // 2.Rejecting high values (eg. > 3m)
    for(int i =0; i < scan_indices; i++)
    {
        if(!std::isfinite(ranges[i]))
        {
            ranges[i] = far_threshold;
            // preprocess for nan or inf
            // nan: ?? how is this happened?
            // inf: it's too far
        }
        else if(ranges[i] > far_threshold)
        {
            // Far enough to consider as gap.
            ranges[i] = far_threshold;
        }
        else if(ranges[i] < near_threshold)
        {
            // Too close to run into
            ranges[i] = near_threshold;
        }
    }

    return;
}

void ReactiveFollowGap::find_gap(float* ranges, std::vector<gap>& gap_array)
{
    gap_threshold = this->get_parameter("gap_threshold").as_double();
    gap_leng_filter = this->get_parameter("gap_leng_filter").as_int();
    gap_maxrange_filter = this->get_parameter("gap_maxrange_filter").as_double();

    gap_array.clear();

    gap temp(0, 0);
    temp.range_sum = ranges[0];
    for(int i = 1; i < scan_indices; i++)
    {
        if(std::abs(ranges[i] - ranges[i-1]) > gap_threshold)
        {
            temp.end_index = i-1;
            temp.length = temp.end_index - temp.start_index;
            gap_array.push_back(temp);
            temp.start_index = i;
            temp.range_sum = 0;
            temp.max_range = 0;
            temp.min_range = 0;
        }
        temp.range_sum += ranges[i];
        temp.max_range = std::max(temp.max_range, ranges[i]);
        temp.min_range = std::min(temp.min_range, ranges[i]);
    }
    temp.end_index = scan_indices - 1;
    temp.length = temp.end_index - temp.start_index;

    gap_array.push_back(temp);
    return;
}

void ReactiveFollowGap::find_max_gap(float* ranges, gap& max_gap)
{
    // Argument
    //  ranges: scan ranges,
    //  indice: max gap
    // Return the start index & end index of the max gap in free_space_ranges

    std::vector<gap> gap_array;

    find_gap(ranges, gap_array);

    visualize_many_gap(ranges, gap_array);

    gap_ver = static_cast<gap_versions>(this->get_parameter("gap_ver").as_int());
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3700,
        "Gap ver: %d(gaps: %zu)", gap_ver, gap_array.size());

    switch (gap_ver)
    {
        case gap_wider:
        {
            int wide_idx = 0;
            float width = 0;
            for (int i = 0; i < gap_array.size(); i++)
            {
                if(width < (gap_array[i].end_index - gap_array[i].start_index) * gap_array[i].range_sum)
                {
                    width = (gap_array[i].end_index - gap_array[i].start_index) * gap_array[i].range_sum;
                    wide_idx = i;
                }
            }
            max_gap = gap_array[wide_idx];

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1300,
                "Wider gap: %.2lf between %d and %d",
                width, max_gap.start_index, max_gap.end_index);

            break;
        }

        case gap_deeper:
        {
            int deep_idx = 0;
            float depth = 0;
            for (int i = 0; i < gap_array.size(); i++)
            {
                if(gap_array[i].end_index <= gap_array[i].start_index)
                {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1700,
                        "Gap index Error from: %d, to: %d", gap_array[i].start_index, gap_array[i].end_index);
                }
                else if(depth < (gap_array[i].range_sum / (gap_array[i].end_index - gap_array[i].start_index)))
                {
                    depth = (gap_array[i].range_sum / (gap_array[i].end_index - gap_array[i].start_index));
                    deep_idx = i;
                }
            }
            max_gap = gap_array[deep_idx];

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1300,
                "Deeper gap: %.2lf between %d and %d",
                depth, max_gap.start_index, max_gap.end_index);

            break;
        }

        case gap_spacier:
        {
            int space_idx = 0;
            float space = 0;
            for (int i = 0; i < gap_array.size(); i++)
            {
                if(space < gap_array[i].range_sum)
                {
                    space = gap_array[i].range_sum;
                    space_idx = i;
                }
            }
            max_gap = gap_array[space_idx];

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1300,
                "Spacier gap: %.2lf between %d and %d",
                space, max_gap.start_index, max_gap.end_index);

            break;
        }
        case gap_combi:
        {
            // gap gain
            space_gain = this->get_parameter("space_gain").as_double();

            int combi_idx = 0;
            float combi = 0;
            for (int i = 0; i < gap_array.size(); i++)
            {
                if(combi < (gap_array[i].range_sum * space_gain + (gap_array[i].end_index - gap_array[i].start_index)* (1- space_gain)))
                {
                    combi = (gap_array[i].range_sum * space_gain + (gap_array[i].end_index - gap_array[i].start_index)* (1- space_gain));
                    combi_idx = i;
                }
            }
            max_gap = gap_array[combi_idx];

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1300,
                "Combi gap: %.2lf between %d and %d",
                combi, max_gap.start_index, max_gap.end_index);

            break;
        }
        default:
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1700, "Gap version doesn't set: %d", gap_ver);
            break;
    }


    return;
}

void ReactiveFollowGap::find_best_point(float* ranges, gap& best_gap)
{
    // Start_i & end_i are start and end indicies of max-gap range, respectively
    // Return index of best point in ranges
    // Naive: Choose the furthest point within ranges and go there

    // apply safety bubble (in incremental way)
    int safety_bubble[2];
    safety_gain = this->get_parameter("safety_gain").as_double();

    // TODO: bubbles needed to be narrower
    safety_bubble[0] = vehicle_width/2 * 4 * 180/M_PI/ranges[best_gap.start_index] * safety_gain;
    safety_bubble[1] = vehicle_width/2 * 4 * 180/M_PI/ranges[best_gap.end_index] * safety_gain;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1700, "Bubbles on Left: %d, Right: %d", safety_bubble[0], safety_bubble[1]);

    int start_i, end_i, best_i;
    start_i = best_gap.start_index + safety_bubble[0];
    end_i = best_gap.end_index - safety_bubble[1];
    best_i = start_i;

    best_ver = this->get_parameter("best_ver").as_int();
    if(start_i > end_i)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 700,
            "Invalid gap from: %d, end: %d(bubble: %d and %d) ", start_i, end_i, safety_bubble[0], safety_bubble[1]);
        // Containment: if happens many time, then change gap version
        // Cause: too narrow gap or bubbles are too big
    }
    else
    {
        switch(best_ver)
        {
            case 0:
            {
                best_i = (end_i + start_i)/2;
                break;
            }
            case 1:
            {
                for (int i = start_i; i < end_i; i++)
                {
                    // deepest point are a lot..
                    // small problem: far end point always best
                    if (ranges[i] > ranges[best_i])
                    {
                        best_i = i;
                    }
                }
                break;
            }
            case 2:
            {
                int deep_start = start_i;
                int deep_end = end_i;
                for (int i = start_i; i < end_i; i++)
                {
                    // deepest point are a lot..
                    if (ranges[i] > ranges[best_i] - 0.01)
                    {
                        if(best_i + 1 == i)
                        {
                            deep_start = best_i;
                        }
                        deep_end = i;
                        best_i = i - 1;
                    }
                }
                best_i = (deep_start + deep_end) / 2;
                break;
            }
            default:
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1700, "Best version doesn't set: %d", best_ver);
                break;
        }

        best_gap.best_index = best_i;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1700,
            "Best ver: %d, point: %d(%d and %d) ", best_ver, best_gap.best_index, start_i, end_i);
    }

    visualize_best_gap(ranges, best_gap);

    return;
}

float ReactiveFollowGap::speed_profile(float gap_width, float gap_depth)
{
    speed = this->get_parameter("speed").as_double();

    // or just lookup table
    return speed;
}

float ReactiveFollowGap::steering_profile(float* ranges, int & index)
{
    // slight modification of pure pursuit
    float dx, dy;
    conversion_coordinate(ranges, index, dx, dy);

    float steering_angle = atan2(2*dy*wheel_base, ranges[index]*ranges[index]);

    visualize_steering(steering_angle);
    return steering_angle;
}


void ReactiveFollowGap::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

    /// TODO:
    // Find closest point to LiDAR
    // Eliminate all points inside 'bubble' (set them to zero)

    // Find max length gap
    float* ranges = const_cast<float*>(scan_msg->ranges.data());
    gap best_gap;

    preprocess_lidar(ranges);
    // Find the best point in the gap
    find_max_gap(ranges, best_gap);
    find_best_point(ranges, best_gap);



    drive_msg.drive.steering_angle = steering_profile(ranges, best_gap.best_index);
    drive_msg.drive.speed = speed_profile(0, 0);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1700,
        "Steer: %.2lf, index: %d ", drive_msg.drive.steering_angle, best_gap.best_index);

    // Publish Drive message
    ackermann_publisher_->publish(drive_msg);
}