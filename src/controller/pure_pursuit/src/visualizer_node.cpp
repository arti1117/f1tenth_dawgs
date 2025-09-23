#include "pure_pursuit/pure_pursuit_node.hpp"

void PurePursuitNode::visualize_waypoint(const std::vector<Waypoint>& wps){

    visualization_msgs::msg::MarkerArray marker_array;

    for(int i =0; i<wps.size(); i++){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.id = i+1;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker.pose.position.x = wps[i].x;
        marker.pose.position.y = wps[i].y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.10;
        marker.scale.y = 0.10;
        marker.scale.z = 0.10;

        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker_array.markers.push_back(marker);
    }

    RCLCPP_INFO_ONCE(this->get_logger(), "Track visualized: %zu", wps.size());

    wp_pub_->publish(marker_array);
}

void PurePursuitNode::visualize_nearest_wp(const std::vector<Waypoint>& wps, const int nearest_idx){

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker.pose.position.x = wps[nearest_idx].x;
    marker.pose.position.y = wps[nearest_idx].y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    wp_near_pub_->publish(marker);
}

void PurePursuitNode::visualize_lookahead_wp(const std::vector<Waypoint>& wps, const double & x, const double & y){

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    marker.color.a = 0.7;
    marker.color.r = 0.8;
    marker.color.g = 0.0;
    marker.color.b = 0.0;;

    wp_ahead_pub_->publish(marker);
}

void PurePursuitNode::visualize_steering(const float speed, const float steering) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego_racecar/base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "steering";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    double length = speed * 0.3;
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

    str_pub_->publish(marker);
}