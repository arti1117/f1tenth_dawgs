#ifndef F1TENTH_PLANNERS_SPEED_OPTIMIZER_HPP
#define F1TENTH_PLANNERS_SPEED_OPTIMIZER_HPP

#include <vector>

namespace f1tenth {

    // Simple speed optimizer: compute feasible max speed along path given curvature and lateral accel limits,
    // then apply forward/backward pass to respect longitudinal accel limits.
    class SpeedOptimizer {
    public:
        SpeedOptimizer(double max_speed, double max_accel, double max_lat_accel, double wheelbase=0.25);
        // path x,y,yaw vectors (same length). returns speed profile (size = path length)
        std::vector<double> solve(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& yaws, double ds);
    private:
        double max_speed_, max_accel_, max_lat_accel_, wheelbase_;
    };


}
#endif