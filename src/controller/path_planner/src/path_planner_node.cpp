#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#ifdef HAS_ACKERMANN
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#endif


#include "path_planner/frenet.hpp"
#include "path_planner/lattice_lut.hpp"
#include "path_planner/utils.hpp"


using std::placeholders::_1;


class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode(): Node("path_planner")
    {
        declare_parameter<std::string>("global_path_topic", "/global_centerline");
        declare_parameter<std::string>("odom_topic", "/odom");
        declare_parameter<std::string>("planned_path_topic", "/planned_path");
        declare_parameter<std::string>("frame_id", "map");
        declare_parameter<bool>("use_lattice", true);
        declare_parameter<bool>("use_frenet", true);
        declare_parameter<double>("planner_horizon", 3.0);

        sub_path_ = create_subscription<nav_msgs::msg::Path>(get_parameter("global_path_topic").as_string(), 1,
        std::bind(&PathPlannerNode::pathCallback, this, _1));
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(get_parameter("odom_topic").as_string(), 30,
        std::bind(&PathPlannerNode::odomCallback, this, _1));


        pub_path_ = create_publisher<nav_msgs::msg::Path>(get_parameter("planned_path_topic").as_string(),10);


        // init planners
        f1tenth::FrenetParams fp;
        fp.road_half_width = 1.2;
        frenet_ = std::make_unique<f1tenth::FrenetPlanner>(fp);


        f1tenth::LatticeConfig lc;
        lc.horizon = get_parameter("planner_horizon").as_double();
        lattice_ = std::make_unique<f1tenth::LatticeLUT>(lc);
        lattice_->build();


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&PathPlannerNode::onTimer, this));
    }


private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        ref_path_ = *msg;
        ref_wps_ = f1tenth::pathMsgToWaypoints(ref_path_);
        if(frenet_) frenet_->set_reference(ref_wps_);
    }


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
    }


    void onTimer()
    {
        if(ref_wps_.size()<5) return;


        // current pose in map frame
        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom_.header; pose.pose = odom_.pose.pose;


        // Convert to Frenet frame
        size_t idx; f1tenth::FrenetState fs;
        double x=pose.pose.position.x, y=pose.pose.position.y;
        if(!frenet_->cart2frenet(x,y,idx,fs)) return;
        fs.ds = std::hypot(odom_.twist.twist.linear.x, odom_.twist.twist.linear.y);


        // Collect simple point obstacles from centerline (none here). Could be from lidar.
        std::vector<std::pair<double,double>> obstacles; // empty for demo


        // Generate frenet candidates
        auto cands = frenet_->generate(fs, obstacles);
        auto best_frenet = frenet_->select_best(cands);


        // Lattice goal sampling at horizon ahead along centerline
        nav_msgs::msg::Path out; out.header.frame_id = get_parameter("frame_id").as_string();


        std::vector<f1tenth::Waypoint> best_seg;


        if(best_frenet){
        // take first second of the frenet path
        for(size_t i=0;i<best_frenet->x.size();++i){
            if(i%2==0){ best_seg.push_back({best_frenet->x[i], best_frenet->y[i], best_frenet->yaw[i], 0.0}); }
            }
        }

        // augment with lattice (goal alignment)
        if(!best_seg.empty()){
            // compute relative goal at horizon
            const auto &p0 = best_seg.front();
            const auto &pH = best_seg.back();
            double dx = pH.x - p0.x;
            double dy = pH.y - p0.y;
            double dth = f1tenth::wrapAngle(pH.yaw - p0.yaw);
            auto sp = lattice_->query(dx,dy,dth);
            if(sp){
                auto loc = lattice_->sample(*sp, 0.05);
                // transform local spiral to global using p0 frame
                double c=std::cos(p0.yaw), s=std::sin(p0.yaw);
                for(auto &w: loc){
                    f1tenth::Waypoint g;
                    g.x = p0.x + c*w.x - s*w.y;
                    g.y = p0.y + s*w.x + c*w.y;
                    g.yaw = f1tenth::wrapAngle(p0.yaw + w.yaw); best_seg.push_back(g);
                }
            }
        }
        // onTimer()

    // Publish
    if(!best_seg.empty()){
        nav_msgs::msg::Path p = f1tenth::waypointsToPathMsg(best_seg, out.header.frame_id);
        p.header.stamp = now();
        pub_path_->publish(p);
    }
}


    // members
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;


    nav_msgs::msg::Path ref_path_;
    std::vector<f1tenth::Waypoint> ref_wps_;
    nav_msgs::msg::Odometry odom_;


    std::unique_ptr<f1tenth::FrenetPlanner> frenet_;
    std::unique_ptr<f1tenth::LatticeLUT> lattice_;


    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}