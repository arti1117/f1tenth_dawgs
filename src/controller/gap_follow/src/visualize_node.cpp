#include "gap_follow/gap_follow.h"

void ReactiveFollowGap::conversion_coordinate(float* ranges, int & index, float & x, float & y) {
    float r, theta;
    r = ranges[index];
    theta = (index - 540)/4.0 * 3.14/180.0;

    conversion_cartesian(r, theta, x, y);
    return;
}
void ReactiveFollowGap::conversion_cartesian(const float & r, const float & theta, float & x, float & y) {

    x = r * cos(theta);
    y = r * sin(theta);
    return;
}

void ReactiveFollowGap::visualize_steering(const float steering) {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "steering";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    double length = 1.0;
    end.x = length * cos(steering);
    end.y = length * sin(steering);
    end.z = 0.0;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    steer_pub_->publish(marker);
}

void ReactiveFollowGap::visualize_many_gap(float* ranges, std::vector<gap>& many_gap) {

    visualization_msgs::msg::MarkerArray marker_array;

    for(int i =0; i<many_gap.size(); i++){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame;
        marker.header.stamp = this->get_clock()->now();
        marker.id = i+1;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        float x0, y0;
        conversion_coordinate(ranges, many_gap[i].start_index, x0, y0);

        marker.pose.position.x = x0;
        marker.pose.position.y = y0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;

        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker_array.markers.push_back(marker);
    }

    RCLCPP_INFO_ONCE(this->get_logger(), "visualize many gap: %zu", many_gap.size());

    many_gap_pub_->publish(marker_array);
}

void ReactiveFollowGap::visualize_best_gap(float* ranges, gap& best_gap) {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame;
    marker.header.stamp = this->get_clock()->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    float x, y;
    conversion_coordinate(ranges, best_gap.best_index, x, y);

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // not scale but coordinate
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    best_gap_pub_->publish(marker);
}