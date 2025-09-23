#pragma once
#include <vector>
#include <optional>
#include <Eigen/Dense>
#include "path_planner/utils.hpp"


namespace f1tenth
{
    struct FrenetState { double s; double d; double ds; double dd; double ddd; };
    struct FrenetTraj {
        std::vector<double> t; // time stamps
        std::vector<double> s; // longitudinal
        std::vector<double> d; // lateral offset
        std::vector<double> x, y, yaw; // Cartesian projection
        double cost{1e9};
        bool collision{false};
    };


    struct FrenetParams {
        double max_speed{5.0};
        double max_accel{4.0};
        double max_curvature{1.0};
        double dt{0.05};
        double maxt{3.0};
        double mint{1.0};
        double target_speed{3.0};
        std::vector<double> d_samples{-1.0,-0.5,0.0,0.5,1.0};
        std::vector<double> t_samples{1.0,1.5,2.0,2.5,3.0};
        double k_j{0.1}, k_t{0.1}, k_d{1.0}, k_v{1.0};
        double safety_radius{0.2};
        double road_half_width{1.0};
    };


    class FrenetPlanner
    {
    public:
        explicit FrenetPlanner(FrenetParams p):p_(p){}


        // Provide centerline in map frame
        void set_reference(const std::vector<Waypoint>& ref){ref_=ref;}


        // Generate candidate trajectories from current frenet state
        std::vector<FrenetTraj> generate(const FrenetState& fs, const std::vector<std::pair<double,double>>& obstacles);


        // Pick best feasible by cost
        std::optional<FrenetTraj> select_best(const std::vector<FrenetTraj>& cands);


        // Cartesian <-> Frenet helpers
        bool cart2frenet(double x, double y, size_t &idx, FrenetState &out) const;
        bool frenet2cart(double s, double d, double &x, double &y, double &yaw) const;


    private:
        FrenetParams p_;
        std::vector<Waypoint> ref_;
    };
}