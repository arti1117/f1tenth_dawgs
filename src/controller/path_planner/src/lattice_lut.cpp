#include "path_planner/lattice_lut.hpp"
#include <cmath>

using namespace f1tenth;

// For simplicity, we "synthesize" spiral parameters by a small heuristic optimizer at build()
// In production, you would solve boundary value problem via gradient descent and store in a file.
void LatticeLUT::build()
{
    table_.clear();
    for(int ix=1; ix<=cfg_.nx; ++ix){
        double dx = ix*cfg_.dx_step;
        for(int iy=-cfg_.ny; iy<=cfg_.ny; ++iy){
            double dy = iy*cfg_.dy_step;
            for(int it=-cfg_.nt; it<=cfg_.nt; ++it){
                double dth = it*cfg_.dtheta_step;
                // naive param guess: length ~ horizon, curvature terms proportional to dy and dth
                SpiralParams sp;
                sp.s = std::min(cfg_.horizon, std::hypot(dx,dy));
                sp.a = 0.0;
                sp.b = 2.0*dy/(sp.s*sp.s + 1e-3);
                sp.c = 6.0*dth/(sp.s*sp.s + 1e-3);
                sp.d = -4.0*dy/(sp.s*sp.s + 1e-3);
                table_[key(ix, iy+cfg_.ny, it+cfg_.nt)] = sp;
            }
        }
    }
}

std::optional<SpiralParams> LatticeLUT::query(double dx, double dy, double dtheta) const
{
    int ix = std::max(1, std::min(cfg_.nx, static_cast<int>(std::round(dx/cfg_.dx_step))));
    int iy = std::max(-cfg_.ny, std::min(cfg_.ny, static_cast<int>(std::round(dy/cfg_.dy_step))));
    int it = std::max(-cfg_.nt, std::min(cfg_.nt, static_cast<int>(std::round(dtheta/cfg_.dtheta_step))));
    auto itb = table_.find(key(ix, iy+cfg_.ny, it+cfg_.nt));
    if(itb==table_.end())
        return std::nullopt;
    else
        return itb->second;
}


std::vector<Waypoint> LatticeLUT::sample(const SpiralParams &sp, double ds) const
{
    std::vector<Waypoint> out; out.reserve(static_cast<size_t>(sp.s/ds)+1);
    double s=0.0; double x=0.0,y=0.0,yaw=0.0;
    while(s<=sp.s+1e-6){
        double u = sp.s<1e-6? 0.0 : s/sp.s;
        double k = sp.a + sp.b*u + sp.c*u*u + sp.d*u*u*u; // curvature
        // unicycle integration step
        yaw += k*ds;
        x += std::cos(yaw)*ds;
        y += std::sin(yaw)*ds;
        out.push_back({x,y,yaw,s});
        s += ds;
    }
    return out;
}