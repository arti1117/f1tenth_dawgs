#pragma once
#include <vector>
#include <optional>
#include <memory>
#include <Eigen/Dense>
#include "path_planner/utils.hpp"
#include "nanoflann.hpp"


namespace f1tenth
{
    // Log levels for debug output
    enum class LogLevel {
        NONE = 0,
        ERROR = 1,
        WARN = 2,
        INFO = 3,
        DEBUG = 4,
        VERBOSE = 5
    };

    struct FrenetState { double s; double d; double ds; double dd; double ddd; };
    struct FrenetTraj {
        std::vector<double> t; // time stamps
        std::vector<double> s; // longitudinal
        std::vector<double> d; // lateral offset
        std::vector<double> x, y, yaw; // Cartesian projection
        std::vector<double> v; // velocity (m/s)
        double cost{1e9};
        bool collision{false};
    };


    struct FrenetParams {
        double max_speed{15.0};
        double max_accel{4.0};
        double max_curvature{1.0};
        double dt{0.05};
        double maxt{3.0};
        double mint{1.0};
        double target_speed{3.0};
        std::vector<double> d_samples{-1.0,-0.5,0.0,0.5,1.0};
        std::vector<double> t_samples{1.0,1.5,2.0,2.5,3.0};
        double k_j{0.1}, k_t{0.1}, k_d{1.0}, k_v{1.0}, k_obs{100.0};
        double safety_radius{0.3};
        double road_half_width{1.0};
        LogLevel log_level{LogLevel::ERROR};  // Default to ERROR only
    };

    // KD-tree wrapper for nanoflann
    struct WaypointCloud {
        std::vector<Waypoint> pts;
        inline size_t kdtree_get_point_count() const { return pts.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            if (dim == 0) return pts[idx].x;
            return pts[idx].y;
        }
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
    };

    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, WaypointCloud>,
        WaypointCloud, 2>;

    class FrenetPlanner
    {
    public:
        explicit FrenetPlanner(FrenetParams p):p_(p), kdtree_(nullptr){}
        ~FrenetPlanner() { if(kdtree_) delete kdtree_; }

        // Provide centerline in map frame
        void set_reference(const std::vector<Waypoint>& ref);


        // Generate candidate trajectories from current frenet state
        std::vector<FrenetTraj> generate(const FrenetState& fs, const std::vector<std::pair<double,double>>& obstacles);


        // Pick best feasible by cost
        std::optional<FrenetTraj> select_best(const std::vector<FrenetTraj>& cands);


        // Cartesian <-> Frenet helpers
        bool cart2frenet(double x, double y, size_t &idx, FrenetState &out) const;
        bool frenet2cart(double s, double d, double &x, double &y, double &yaw) const;

        // Lattice LUT support functions
        size_t ref_size() const { return ref_.size(); }
        double curvature_at_s(double s) const;


    private:
        FrenetParams p_;
        std::vector<Waypoint> ref_;
        WaypointCloud cloud_;
        KDTree* kdtree_;
        bool is_closed_loop_;
        double total_length_;

        void build_kdtree();
        size_t find_nearest_waypoint(double x, double y) const;
    };
}