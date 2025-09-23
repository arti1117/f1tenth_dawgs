#pragma once
#include <unordered_map>
#include <vector>
#include <tuple>
#include <optional>
#include <Eigen/Dense>
#include "path_planner/utils.hpp"


namespace f1tenth
{
// Cubic spiral params (a,b,c,d,s) for curvature k(s) = a + b*u + c*u^2 + d*u^3, u in [0,1]
    struct SpiralParams{ double a,b,c,d,s; };


    struct LatticeConfig{
    double horizon{3.0};
    double dx_step{0.5};
    double dy_step{0.25};
    double dtheta_step{M_PI/18.0};
    int nx{6}; // forward cells along horizon
    int ny{9}; // lateral cells
    int nt{7}; // heading cells
    };


    class LatticeLUT
    {
    public:
        explicit LatticeLUT(const LatticeConfig &cfg):cfg_(cfg){}

        // Build or load a dense lookup of goal -> spiral params
        void build();

        // Query by relative goal (dx,dy,dtheta) quantized
        std::optional<SpiralParams> query(double dx, double dy, double dtheta) const;

        // Sample path points from spiral params in local frame
        std::vector<Waypoint> sample(const SpiralParams &sp, double ds=0.05) const;


    private:
        LatticeConfig cfg_;
        std::unordered_map<long long, SpiralParams> table_;
        long long key(int ix,int iy,int it) const {
            return (static_cast<long long>(ix)&0x3FF) |
            ((static_cast<long long>(iy)&0x3FF)<<10) | ((static_cast<long long>(it)&0x3FF)<<20);
        }
    };
}