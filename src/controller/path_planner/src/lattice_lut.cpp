#include "path_planner/lattice_lut.hpp"
#include "path_planner/frenet.hpp"
#include <cmath>
#include <iostream>

using namespace f1tenth;

// Logging macro
#define LATTICE_LOG(level, msg) do { \
    if (cfg_.log_level >= level) { \
        std::cout << msg << std::endl; \
    } \
} while(0)

/**
 * @brief Build Frenet-based Lattice LUT using reference path curvature
 * Each LUT entry stores polynomial parameters for a local spiral (s, d, yaw)
 * generated relative to the Frenet frame of the reference centerline.
 */
void LatticeLUT::build(const FrenetPlanner& frenet)
{
    table_.clear();
    LATTICE_LOG(LogLevel::INFO, "[LatticeLUT] Building Frenet-based LUT...");

    for (int ix = 1; ix <= cfg_.nx; ++ix) {
        double ds = ix * cfg_.dx_step;  // forward distance along centerline

        for (int iy = -cfg_.ny; iy <= cfg_.ny; ++iy) {
            double d = iy * cfg_.dy_step;  // lateral offset

            for (int it = -cfg_.nt; it <= cfg_.nt; ++it) {
                double dth = it * cfg_.dtheta_step;  // heading offset

                SpiralParams sp;
                sp.s = std::min(cfg_.horizon, ds);

                // get curvature from centerline at s=0
                double k_center = 0.0;
                if (frenet.ref_size() >= 2)
                    k_center = frenet.curvature_at_s(0.0);  // 구현 필요 (보통 central diff)

                // polynomial curvature profile around centerline curvature
                sp.a = k_center;
                sp.b = (2.0 * d) / (sp.s * sp.s + 1e-3);
                sp.c = (6.0 * dth) / (sp.s * sp.s + 1e-3);
                sp.d = (-4.0 * d) / (sp.s * sp.s + 1e-3);

                table_[key(ix, iy + cfg_.ny, it + cfg_.nt)] = sp;
            }
        }
    }

    LATTICE_LOG(LogLevel::DEBUG, "[LatticeLUT] Built " << table_.size() << " entries");
}


/**
 * @brief Query nearest LUT entry by dx, dy, dtheta
 */
std::optional<SpiralParams> LatticeLUT::query(double dx, double dy, double dtheta) const
{
    int ix = std::max(1, std::min(cfg_.nx, static_cast<int>(std::round(dx / cfg_.dx_step))));
    int iy = std::max(-cfg_.ny, std::min(cfg_.ny, static_cast<int>(std::round(dy / cfg_.dy_step))));
    int it = std::max(-cfg_.nt, std::min(cfg_.nt, static_cast<int>(std::round(dtheta / cfg_.dtheta_step))));

    auto itb = table_.find(key(ix, iy + cfg_.ny, it + cfg_.nt));
    if (itb == table_.end()) {
        LATTICE_LOG(LogLevel::ERROR, "[LatticeLUT] query failed for (" << dx << "," << dy << "," << dtheta << ")");
        return std::nullopt;
    }

    return itb->second;
}


/**
 * @brief Sample global waypoints from spiral parameters using Frenet frame
 *        Starting from current position (s_start, d_start) in Frenet coordinates
 */
std::vector<Waypoint> LatticeLUT::sample(const SpiralParams &sp, double ds, const FrenetPlanner &frenet,
                                          double s_start, double d_start) const
{
    std::vector<Waypoint> out;
    out.reserve(static_cast<size_t>(sp.s / ds) + 1);

    double s_local = 0.0;
    double d_integrated = d_start;  // Start from current lateral offset

    while (s_local <= sp.s + 1e-6) {
        double u = sp.s < 1e-6 ? 0.0 : s_local / sp.s;

        // Curvature profile
        double k = sp.a + sp.b * u + sp.c * u * u + sp.d * u * u * u;

        // Lateral offset profile (integrate curvature effect from initial d_start)
        d_integrated = d_start + sp.b * u * u * sp.s / 2.0 + sp.c * u * u * u * sp.s / 3.0;

        // Frenet coordinates (s along reference path from current position, d lateral offset)
        double s_frenet = s_start + s_local;  // Start from current s position
        double d_frenet = d_integrated;

        // Convert to global coordinates using Frenet transformation
        double gx, gy, gyaw;
        if (!frenet.frenet2cart(s_frenet, d_frenet, gx, gy, gyaw)) {
            LATTICE_LOG(LogLevel::ERROR, "[LatticeLUT] frenet2cart failed at s=" << s_frenet << ", d=" << d_frenet);
            break;
        }

        // Add curvature effect to yaw
        gyaw += k * ds * u;

        out.push_back({gx, gy, gyaw, s_frenet});
        s_local += ds;
    }

    LATTICE_LOG(LogLevel::DEBUG, "[LatticeLUT] Sampled " << out.size() << " pts from spiral (s_start=" << s_start
                                  << ", d_start=" << d_start << ", s_length=" << sp.s << ")");
    return out;
}
