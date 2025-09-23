#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <utility>

#include <math.h>
#include <vector>
#include <deque>
#include <map>
#include <iostream>
#include <sstream>
#include <algorithm>


class IahrsTfPublisher : public rclcpp::Node
{
public:
    IahrsTfPublisher()
    : Node("iahrs_tf_publisher")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/iahrs/imu",
        rclcpp::SensorDataQoS(),
        std::bind(&IahrsTfPublisher::imuCallback, this, std::placeholders::_1)
        );

    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "base";
        transformStamped.child_frame_id = "iahrs";

        // only rotation from imu
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation = msg->orientation;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<IahrsTfPublisher>());
    rclcpp::shutdown();



    return 0;
}