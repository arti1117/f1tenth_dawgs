#include "path_planner/speed_optimizer.hpp"
#include <cmath>
#include <algorithm>


using namespace f1tenth;


SpeedOptimizer::SpeedOptimizer(double max_speed, double max_accel, double max_lat_accel, double wheelbase)
: max_speed_(max_speed), max_accel_(max_accel), max_lat_accel_(max_lat_accel), wheelbase_(wheelbase){}


static double curvature_from_yaw_pair(double yaw_prev, double yaw_next, double ds){
    double dyaw = yaw_next - yaw_prev;
    while(dyaw > M_PI) dyaw -= 2*M_PI;
    while(dyaw < -M_PI) dyaw += 2*M_PI;
    return dyaw / std::max(1e-6, ds);
}


std::vector<double> SpeedOptimizer::solve(
    const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& yaws, double ds){
    size_t n = xs.size();
    std::vector<double> vmax(n, max_speed_);
    // curvature-based lateral constraint: v <= sqrt(a_lat_max / |kappa|)
    for(size_t i=0;i+1<n;i++){
        double k = std::abs(curvature_from_yaw_pair(yaws[i], yaws[std::min(n-1,i+1)], ds));
        if(k > 1e-6) vmax[i] = std::min(vmax[i], std::sqrt(max_lat_accel_ / k));
    }
    // forward pass for accel
    std::vector<double> v(n,0.0);
    v[0] = std::min(vmax[0], max_speed_);
    for(size_t i=1;i<n;i++){
        double v_lim = std::sqrt(v[i-1]*v[i-1] + 2*max_accel_*ds);
        v[i] = std::min(vmax[i], v_lim);
    }
    // backward pass for decel
    for(int i=(int)n-2;i>=0;--i){
        double v_lim = std::sqrt(v[i+1]*v[i+1] + 2*max_accel_*ds);
        v[i] = std::min(v[i], v_lim);
    }
    return v;
}