#pragma once
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace f1tenth
{
    struct Waypoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double s{0.0}; // cumulative arc-length
    double v{0.0}; // velocity (m/s)
    };

    inline double wrapAngle(double a)
    {
        while (a > M_PI) a -= 2*M_PI;
        while (a < -M_PI) a += 2*M_PI;
        return a;
    }

    inline double distance(double x1,double y1,double x2,double y2){
        const double dx=x1-x2, dy=y1-y2;
        return std::hypot(dx,dy);
    }


    inline std::vector<Waypoint> pathMsgToWaypoints(const nav_msgs::msg::Path &path)
    {
        std::vector<Waypoint> wps; wps.reserve(path.poses.size());
        double s_acc=0.0;
        for (size_t i=0;i<path.poses.size();++i){
            const auto &p = path.poses[i].pose.position;
            double yaw=0.0;
            if(i+1<path.poses.size()){
                const auto &p2 = path.poses[i+1].pose.position;
                yaw = std::atan2(p2.y - p.y, p2.x - p.x);
            }
            else if(i>0){
                const auto &p2 = path.poses[i-1].pose.position;
                yaw = std::atan2(p.y - p2.y, p.x - p2.x);
            }
            if(i>0){
                const auto &pp = path.poses[i-1].pose.position;
                s_acc += std::hypot(p.x-pp.x,p.y-pp.y);
            }
            wps.push_back({p.x,p.y,yaw,s_acc});
        }
        return wps;
    }


    inline nav_msgs::msg::Path waypointsToPathMsg(const std::vector<Waypoint> &wps, const std::string &frame)
    {
        nav_msgs::msg::Path path; path.header.frame_id=frame; path.poses.resize(wps.size());
        for (size_t i=0;i<wps.size();++i){
            path.poses[i].pose.position.x = wps[i].x;
            path.poses[i].pose.position.y = wps[i].y;
            path.poses[i].pose.position.z = wps[i].v;  // Store velocity in z component

            // Set orientation based on yaw
            path.poses[i].pose.orientation.x = 0.0;
            path.poses[i].pose.orientation.y = 0.0;
            path.poses[i].pose.orientation.z = std::sin(wps[i].yaw / 2.0);
            path.poses[i].pose.orientation.w = std::cos(wps[i].yaw / 2.0);
        }
        return path;
    }
}