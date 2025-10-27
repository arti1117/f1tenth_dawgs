#include "path_planner/frenet.hpp"
#include <limits>
#include <algorithm>
#include <iostream>
#include <sstream>

using namespace f1tenth;

// Logging macro
#define FRENET_LOG(level, msg) do { \
    if (p_.log_level >= level) { \
        std::cout << msg << std::endl; \
    } \
} while(0)

void FrenetPlanner::set_reference(const std::vector<Waypoint>& ref)
{
    ref_ = ref;
    if (ref_.empty()) {
        FRENET_LOG(LogLevel::ERROR, "[Frenet] ERROR: Reference path is empty!");
        return;
    }

    FRENET_LOG(LogLevel::INFO, "[Frenet] Setting reference path with " << ref_.size() << " waypoints");
    FRENET_LOG(LogLevel::DEBUG, "[Frenet] Path bounds: x=[" << ref_.front().x << ", " << ref_.back().x
              << "] y=[" << ref_.front().y << ", " << ref_.back().y << "]");

    // Check if path is closed
    double first_last_dist = distance(ref_.front().x, ref_.front().y, ref_.back().x, ref_.back().y);
    is_closed_loop_ = (first_last_dist < 2.0);

    FRENET_LOG(LogLevel::DEBUG, "[Frenet] First-last distance: " << first_last_dist << " m");
    FRENET_LOG(LogLevel::INFO, "[Frenet] Path is " << (is_closed_loop_ ? "CLOSED LOOP" : "OPEN"));

    // Calculate total length
    total_length_ = ref_.back().s;
    if (is_closed_loop_) {
        total_length_ += first_last_dist;
    }

    FRENET_LOG(LogLevel::INFO, "[Frenet] Total path length: " << total_length_ << " m");
    FRENET_LOG(LogLevel::DEBUG, "[Frenet] Last waypoint s-coordinate: " << ref_.back().s << " m");

    // Build KD-tree
    build_kdtree();
}

void FrenetPlanner::build_kdtree()
{
    if (kdtree_) {
        delete kdtree_;
        kdtree_ = nullptr;
    }

    cloud_.pts.clear();
    cloud_.pts = ref_;  // Copy waypoints to cloud

    if (cloud_.pts.empty()) {
        FRENET_LOG(LogLevel::ERROR, "[Frenet] ERROR: Cannot build KD-tree with empty waypoints!");
        return;
    }

    FRENET_LOG(LogLevel::DEBUG, "[Frenet] Building KD-tree with " << cloud_.pts.size() << " points");

    // Build KD-tree
    kdtree_ = new KDTree(2 /*dim*/, cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10 /*max leaf*/));
    kdtree_->buildIndex();

    FRENET_LOG(LogLevel::DEBUG, "[Frenet] KD-tree built successfully");
}

size_t FrenetPlanner::find_nearest_waypoint(double x, double y) const
{
    if (!kdtree_ || ref_.empty()) {
        FRENET_LOG(LogLevel::WARN, "[Frenet] WARNING: KD-tree not initialized or ref empty, returning 0");
        return 0;
    }

    // Query point
    double query_pt[2] = {x, y};

    // Search for nearest neighbor
    size_t nearest_idx;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&nearest_idx, &out_dist_sqr);
    kdtree_->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParameters(10));

    double dist = std::sqrt(out_dist_sqr);
    FRENET_LOG(LogLevel::VERBOSE, "[Frenet] Nearest waypoint to (" << x << ", " << y << ") is idx="
              << nearest_idx << " at (" << ref_[nearest_idx].x << ", "
              << ref_[nearest_idx].y << ") dist=" << dist << " m");

    return nearest_idx;
}

bool FrenetPlanner::cart2frenet(double x, double y, size_t &best_seg_idx, FrenetState &out) const
{
  FRENET_LOG(LogLevel::VERBOSE, "\n[Frenet] ===== cart2frenet conversion =====");
  FRENET_LOG(LogLevel::VERBOSE, "[Frenet] Input: x=" << x << ", y=" << y);

  // require at least 2 waypoints
  if (ref_.size() < 2 || !kdtree_) {
    FRENET_LOG(LogLevel::ERROR, "[Frenet] ERROR: ref size=" << ref_.size()
              << ", kdtree=" << (kdtree_ ? "valid" : "null"));
    return false;
  }

  FRENET_LOG(LogLevel::DEBUG, "[Frenet] Path has " << ref_.size() << " waypoints, closed="
            << is_closed_loop_ << ", total_length=" << total_length_);

  // Use KD-tree to find nearest waypoint quickly
  size_t nearest_idx = find_nearest_waypoint(x, y);

  // Local search window around nearest point
  const int search_window = 5;  // Search ±5 segments around nearest point

  double best_dist = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  double best_d = 0.0;
  bool found = false;

  FRENET_LOG(LogLevel::DEBUG, "[Frenet] Searching ±" << search_window << " segments around nearest idx="
            << nearest_idx);

  // Search locally around nearest point for best projection
  // This is more efficient than searching all segments
  for (int offset = -search_window; offset <= search_window; ++offset) {
    int i = static_cast<int>(nearest_idx) + offset;

    // Handle wrapping for closed loops
    if (is_closed_loop_) {
      i = (i + ref_.size()) % ref_.size();
    } else {
      if (i < 0 || i >= static_cast<int>(ref_.size()) - 1) {
        FRENET_LOG(LogLevel::VERBOSE, "[Frenet] Skipping out-of-bounds segment idx=" << i);
        continue;
      }
    }

    size_t j = (i + 1) % ref_.size();  // Next index with wrapping

    FRENET_LOG(LogLevel::VERBOSE, "[Frenet] Checking segment " << i << "->" << j
              << " (" << ref_[i].x << "," << ref_[i].y << ") -> ("
              << ref_[j].x << "," << ref_[j].y << ")");

    double x1 = ref_[i].x, y1 = ref_[i].y;
    double x2 = ref_[j].x, y2 = ref_[j].y;
    double vx = x2 - x1;
    double vy = y2 - y1;
    double seg_norm2 = vx*vx + vy*vy;

    if (seg_norm2 < 1e-9) continue;  // Skip degenerate segments

    // Project point onto segment
    double t = ((x - x1) * vx + (y - y1) * vy) / seg_norm2;
    t = std::max(0.0, std::min(1.0, t));  // Clamp to [0,1]

    double px = x1 + t * vx;
    double py = y1 + t * vy;
    double dist = std::hypot(x - px, y - py);

    FRENET_LOG(LogLevel::VERBOSE, "[Frenet]   Projection: t=" << t << ", proj=(" << px << "," << py
              << "), dist=" << dist << " m");

    if (dist < best_dist) {
      best_dist = dist;
      best_seg_idx = i;

      // Calculate s coordinate
      double s_at_i = ref_[i].s;
      double proj_dist = t * std::sqrt(seg_norm2);
      best_s = s_at_i + proj_dist;

      FRENET_LOG(LogLevel::DEBUG, "[Frenet]   NEW BEST! s_at_i=" << s_at_i << ", proj_dist="
                << proj_dist << ", best_s=" << best_s);

      // For closed loops, handle wrapping when on the closing segment
      if (is_closed_loop_ && static_cast<size_t>(i) == ref_.size() - 1) {
        // On closing segment (last->first)
        best_s = ref_.back().s + proj_dist;
        FRENET_LOG(LogLevel::DEBUG, "[Frenet]   Closing segment detected! Adjusted best_s=" << best_s);
        // Wrap s if it exceeds total length
        if (best_s >= total_length_) {
          best_s = std::fmod(best_s, total_length_);
          FRENET_LOG(LogLevel::DEBUG, "[Frenet]   Wrapped s to " << best_s);
        }
      }

      // Calculate signed lateral distance
      double seg_yaw = std::atan2(vy, vx);
      double nx = -std::sin(seg_yaw);
      double ny = std::cos(seg_yaw);
      best_d = (x - px) * nx + (y - py) * ny;

      FRENET_LOG(LogLevel::VERBOSE, "[Frenet]   Lateral: seg_yaw=" << seg_yaw
                << " rad, best_d=" << best_d << " m");
      found = true;
    }
  }

  if (!found) {
    FRENET_LOG(LogLevel::ERROR, "[Frenet] ERROR: No valid projection found!");
    return false;
  }

  // For closed loops, ensure s is wrapped to [0, total_length)
  if (is_closed_loop_ && total_length_ > 0) {
    double original_s = best_s;
    best_s = std::fmod(best_s + total_length_, total_length_);  // Handle negative s
    if (original_s != best_s) {
      FRENET_LOG(LogLevel::DEBUG, "[Frenet] Final wrapping: " << original_s << " -> " << best_s);
    }
  }

  // Fill out frenet state
  out.s = best_s;
  out.d = best_d;
  out.ds = 0.0;
  out.dd = 0.0;
  out.ddd = 0.0;

  FRENET_LOG(LogLevel::DEBUG, "[Frenet] SUCCESS: s=" << out.s << ", d=" << out.d
            << ", seg_idx=" << best_seg_idx << ", dist=" << best_dist << " m");
  FRENET_LOG(LogLevel::VERBOSE, "[Frenet] ===== cart2frenet complete =====\n");

  return true;
}

bool FrenetPlanner::frenet2cart(double s, double d, double &x, double &y, double &yaw) const
{
  FRENET_LOG(LogLevel::VERBOSE, "\n[Frenet] ===== frenet2cart conversion =====");
  FRENET_LOG(LogLevel::VERBOSE, "[Frenet] Input: s=" << s << ", d=" << d);

  if (ref_.size() < 2) {
    FRENET_LOG(LogLevel::ERROR, "[Frenet] ERROR: ref size=" << ref_.size() << " (need at least 2)");
    return false;
  }

  // For closed loops, wrap s to [0, total_length)
  double s_wrapped = s;
  if (is_closed_loop_ && total_length_ > 0) {
    s_wrapped = std::fmod(s, total_length_);
    if (s_wrapped < 0) s_wrapped += total_length_;
    if (s != s_wrapped) {
      FRENET_LOG(LogLevel::DEBUG, "[Frenet] Wrapped s: " << s << " -> " << s_wrapped
                << " (total_length=" << total_length_ << ")");
    }
  }

  // Handle closing segment for closed paths
  double first_last_dist = is_closed_loop_ ? distance(ref_.front().x, ref_.front().y, ref_.back().x, ref_.back().y) : 0;
  FRENET_LOG(LogLevel::DEBUG, "[Frenet] Closed loop: " << is_closed_loop_
            << ", last_s=" << ref_.back().s
            << ", first_last_dist=" << first_last_dist);

  if (is_closed_loop_ && s_wrapped >= ref_.back().s) {
    FRENET_LOG(LogLevel::DEBUG, "[Frenet] Point is on closing segment (last->first)");
    // Point is on the closing segment (last -> first)
    double s_on_closing = s_wrapped - ref_.back().s;
    double closing_length = first_last_dist;

    if (closing_length > 1e-6) {
      double r = s_on_closing / closing_length;
      r = std::max(0.0, std::min(1.0, r));

      const auto &p0 = ref_.back();
      const auto &p1 = ref_.front();

      double cx = p0.x + r * (p1.x - p0.x);
      double cy = p0.y + r * (p1.y - p0.y);
      double cyaw = std::atan2(p1.y - p0.y, p1.x - p0.x);

      // Apply lateral offset
      double nx = -std::sin(cyaw);
      double ny = std::cos(cyaw);
      x = cx + nx * d;
      y = cy + ny * d;
      yaw = cyaw;

      FRENET_LOG(LogLevel::DEBUG, "[Frenet] Closing segment result: x=" << x << ", y=" << y
                << ", yaw=" << yaw << " rad");
      FRENET_LOG(LogLevel::VERBOSE, "[Frenet] ===== frenet2cart complete =====\n");
      return true;
    }
  }

  // Find segment containing s by linear search
  size_t idx = 0;
  if (s_wrapped <= ref_.front().s) {
    idx = 0;
    FRENET_LOG(LogLevel::VERBOSE, "[Frenet] s_wrapped <= front.s, using idx=0");
  } else if (s_wrapped >= ref_.back().s) {
    idx = std::max(0, static_cast<int>(ref_.size()) - 2);
    FRENET_LOG(LogLevel::VERBOSE, "[Frenet] s_wrapped >= back.s, using idx=" << idx);
  } else {
    for (size_t i = 0; i + 1 < ref_.size(); ++i) {
      if (ref_[i].s <= s_wrapped && s_wrapped <= ref_[i+1].s) {
        idx = i;
        FRENET_LOG(LogLevel::VERBOSE, "[Frenet] Found segment: idx=" << idx
                  << " [" << ref_[i].s << ", " << ref_[i+1].s << "]");
        break;
      }
    }
  }

  const auto &p0 = ref_[idx];
  const auto &p1 = ref_[(idx + 1) % ref_.size()];
  double ds_seg = (idx + 1 < ref_.size()) ? (p1.s - p0.s) : first_last_dist;

  if (std::abs(ds_seg) < 1e-6) {
    x = p0.x;
    y = p0.y;
    yaw = p0.yaw;
    return true;
  }

  double r = (s_wrapped - p0.s) / ds_seg;
  r = std::max(0.0, std::min(1.0, r));

  double cx = p0.x + r * (p1.x - p0.x);
  double cy = p0.y + r * (p1.y - p0.y);
  double cyaw = std::atan2(p1.y - p0.y, p1.x - p0.x);

  // Apply lateral offset
  double nx = -std::sin(cyaw);
  double ny = std::cos(cyaw);
  x = cx + nx * d;
  y = cy + ny * d;
  yaw = cyaw;

  FRENET_LOG(LogLevel::DEBUG, "[Frenet] Normal segment result: x=" << x << ", y=" << y
            << ", yaw=" << yaw << " rad");
  FRENET_LOG(LogLevel::VERBOSE, "[Frenet] Segment " << idx << "->" << (idx+1) % ref_.size()
            << ", r=" << r << ", center=(" << cx << "," << cy << ")");
  FRENET_LOG(LogLevel::VERBOSE, "[Frenet] ===== frenet2cart complete =====\n");
  return true;
}

double FrenetPlanner::curvature_at_s(double s) const
{
    if (ref_.size() < 3) return 0.0;  // Need at least 3 points for curvature

    // Wrap s for closed loops
    double s_wrapped = s;
    if (is_closed_loop_ && total_length_ > 0) {
        s_wrapped = std::fmod(s, total_length_);
        if (s_wrapped < 0) s_wrapped += total_length_;
    }

    // Find segment containing s
    size_t idx = 0;
    if (s_wrapped <= ref_.front().s) {
        idx = 0;
    } else if (s_wrapped >= ref_.back().s) {
        idx = std::max(0, static_cast<int>(ref_.size()) - 2);
    } else {
        for (size_t i = 0; i + 1 < ref_.size(); ++i) {
            if (ref_[i].s <= s_wrapped && s_wrapped <= ref_[i+1].s) {
                idx = i;
                break;
            }
        }
    }

    // Use 3-point curvature estimation
    size_t i0 = (idx == 0) ? (is_closed_loop_ ? ref_.size() - 1 : 0) : idx - 1;
    size_t i1 = idx;
    size_t i2 = (idx + 1 >= ref_.size()) ? (is_closed_loop_ ? 0 : ref_.size() - 1) : idx + 1;

    // Get three points
    double x0 = ref_[i0].x, y0 = ref_[i0].y;
    double x1 = ref_[i1].x, y1 = ref_[i1].y;
    double x2 = ref_[i2].x, y2 = ref_[i2].y;

    // Calculate curvature using formula: k = 2 * |cross(v1, v2)| / |v1|^3
    double v1x = x1 - x0, v1y = y1 - y0;
    double v2x = x2 - x1, v2y = y2 - y1;

    double cross = v1x * v2y - v1y * v2x;
    double v1_mag = std::sqrt(v1x * v1x + v1y * v1y);

    if (v1_mag < 1e-6) return 0.0;

    double curvature = 2.0 * std::abs(cross) / (v1_mag * v1_mag * v1_mag);

    // Add sign based on turning direction
    if (cross < 0) curvature = -curvature;

    FRENET_LOG(LogLevel::VERBOSE, "[Frenet] Curvature at s=" << s << ": k=" << curvature);

    return curvature;
}

std::vector<FrenetTraj> FrenetPlanner::generate(
    const FrenetState& fs,
    const std::vector<std::pair<double,double>>& obstacles)
{
    std::vector<FrenetTraj> cands;
    if (ref_.empty()) return cands;

    // DEBUG: Log number of obstacles received
    FRENET_LOG(LogLevel::DEBUG, "[Frenet] Received " << obstacles.size() << " obstacle points");
    if (!obstacles.empty()) {
        FRENET_LOG(LogLevel::DEBUG, "[Frenet] First obstacle at: (" << obstacles[0].first << ", " << obstacles[0].second << ")");
        FRENET_LOG(LogLevel::DEBUG, "[Frenet] Safety parameters: base_radius=" << p_.safety_radius
                  << ", vehicle_r=" << p_.vehicle_radius << ", obstacle_r=" << p_.obstacle_radius);
    }

    // == Parameter sampling ==
    for (double T : p_.t_samples) {
        for (double df : p_.d_samples) {
            // === Lateral quintic (d) ===
            // boundary: (d, d', d'') -> (df, 0, 0)
            Eigen::Matrix3d A;
            A << pow(T, 3), pow(T, 4), pow(T, 5),
                 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
                 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
            Eigen::Vector3d b;
            b << df - (fs.d + fs.dd * T + 0.5 * fs.ddd * T * T),
                 - (fs.dd + fs.ddd * T),
                 - fs.ddd;
            Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

            // corrected initial condition mapping
            double a0 = fs.d;      // position
            double a1 = fs.dd;     // d'(0)
            double a2 = 0.5 * fs.ddd; // d''(0)/2
            double a3 = x(0);
            double a4 = x(1);
            double a5 = x(2);

            // === Longitudinal quartic (s) ===
            // boundary: (s, s', s'') -> (s + v*T, v, 0)
            double v = p_.target_speed;
            double s_ddot_init = 0.0;  // assume zero if unavailable

            Eigen::Matrix2d As;
            As << 3 * pow(T, 2), 4 * pow(T, 3),
                  6 * T, 12 * pow(T, 2);
            Eigen::Vector2d bs;
            bs << (v - (fs.ds + s_ddot_init * T)),
                   - s_ddot_init;
            Eigen::Vector2d xs = As.colPivHouseholderQr().solve(bs);

            double b0 = fs.s;
            double b1 = fs.ds;
            double b2 = 0.5 * s_ddot_init;
            double b3 = xs(0);
            double b4 = xs(1);

            // === Trajectory sampling ===
            FrenetTraj tr;
            tr.collision = false;
            tr.cost = 0.0;

            for (double t = 0.0; t <= T + 1e-6; t += p_.dt) {
                // Longitudinal motion
                double s = b0 + b1 * t + b2 * t * t + b3 * t * t * t + b4 * t * t * t * t;
                double s_dot = b1 + 2 * b2 * t + 3 * b3 * t * t + 4 * b4 * t * t * t;

                // Lateral motion
                double d = a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
                double d_dot = a1 + 2 * a2 * t + 3 * a3 * t * t + 4 * a4 * t * t * t + 5 * a5 * t * t * t * t;
                double d_ddot = 2 * a2 + 6 * a3 * t + 12 * a4 * t * t + 20 * a5 * t * t * t;

                // sanity checks
                if (std::abs(d_ddot) > p_.max_accel * 2.0) { tr.cost = 1e9; break; }
                if (s_dot < 0.0 || s_dot > p_.max_speed * 1.5) { tr.cost = 1e9; break; }

                // wrap s for closed loops
                double s_wrapped = s;
                if (is_closed_loop_ && total_length_ > 0) {
                    s_wrapped = std::fmod(s, total_length_);
                    if (s_wrapped < 0) s_wrapped += total_length_;
                }

                // Convert Frenet → Cartesian
                double xg, yg, yawg;
                if (!frenet2cart(s_wrapped, d, xg, yg, yawg)) {
                    tr.cost = 1e9; break;
                }

                tr.t.push_back(t);
                tr.s.push_back(s_wrapped);
                tr.d.push_back(d);
                tr.x.push_back(xg);
                tr.y.push_back(yg);
                tr.yaw.push_back(yawg);
                tr.v.push_back(s_dot);  // Store longitudinal velocity
            }

            // === Enhanced Collision Check ===
            double proximity_cost = 0.0;  // Initialize proximity cost
            size_t collision_checks = 0;  // Track number of collision checks

            if (tr.x.size() > 1) {
                for (size_t i = 0; i < tr.x.size(); ++i) {
                    // Velocity-dependent safety margin
                    // safety_margin = base + vehicle_r + obstacle_r + k_v * velocity
                    double current_velocity = (i < tr.v.size()) ? tr.v[i] : p_.target_speed;
                    double dynamic_safety = std::max(p_.min_safety_margin,
                                                     p_.safety_radius + p_.vehicle_radius + p_.obstacle_radius +
                                                     p_.k_velocity_safety * current_velocity);

                    FRENET_LOG(LogLevel::VERBOSE, "[Frenet] Point " << i << ": v=" << current_velocity
                              << " m/s, safety=" << dynamic_safety << " m");

                    // Check collision with obstacles
                    for (const auto &ob : obstacles) {
                        collision_checks++;
                        double dist = distance(tr.x[i], tr.y[i], ob.first, ob.second);

                        // Hard collision check
                        if (dist < dynamic_safety) {
                            tr.collision = true;
                            FRENET_LOG(LogLevel::DEBUG, "[Frenet] COLLISION at point " << i
                                      << ": dist=" << dist << " < safety=" << dynamic_safety);
                            break;
                        }

                        // Proximity cost (soft penalty for being close to obstacles)
                        if (dist < p_.proximity_threshold) {
                            double margin = dist - dynamic_safety;
                            if (margin > 0) {
                                // Add inverse distance cost (higher cost when closer)
                                proximity_cost += 1.0 / (margin + 0.1);
                            }
                        }
                    }

                    // Road boundary check
                    if (std::abs(tr.d[i]) > p_.road_half_width) {
                        tr.collision = true;
                        FRENET_LOG(LogLevel::DEBUG, "[Frenet] OUT OF BOUNDS at point " << i
                                  << ": d=" << tr.d[i] << " > width=" << p_.road_half_width);
                        break;
                    }

                    if (tr.collision) break;

                    // Interpolation check (check intermediate points between samples)
                    if (i > 0 && p_.interpolation_checks > 0) {
                        for (int j = 1; j <= p_.interpolation_checks; ++j) {
                            double t_interp = static_cast<double>(j) / (p_.interpolation_checks + 1);
                            double x_interp = tr.x[i-1] + t_interp * (tr.x[i] - tr.x[i-1]);
                            double y_interp = tr.y[i-1] + t_interp * (tr.y[i] - tr.y[i-1]);
                            double d_interp = tr.d[i-1] + t_interp * (tr.d[i] - tr.d[i-1]);

                            // Check interpolated points
                            for (const auto &ob : obstacles) {
                                double dist_interp = distance(x_interp, y_interp, ob.first, ob.second);
                                if (dist_interp < dynamic_safety) {
                                    tr.collision = true;
                                    FRENET_LOG(LogLevel::DEBUG, "[Frenet] INTERPOLATED COLLISION between points "
                                              << (i-1) << "-" << i << ", interp " << j);
                                    break;
                                }
                            }

                            // Road boundary for interpolated points
                            if (std::abs(d_interp) > p_.road_half_width) {
                                tr.collision = true;
                                FRENET_LOG(LogLevel::DEBUG, "[Frenet] INTERPOLATED OUT OF BOUNDS: d="
                                          << d_interp);
                                break;
                            }

                            if (tr.collision) break;
                        }
                    }

                    if (tr.collision) break;
                }
            }

            // === Cost computation ===
            if (!tr.collision && tr.x.size() > 1) {
                double j_lat = 0.0;
                for (size_t i = 2; i < tr.d.size(); ++i) {
                    j_lat += std::abs(tr.d[i] - 2 * tr.d[i - 1] + tr.d[i - 2]);
                }
                double dev = std::accumulate(tr.d.begin(), tr.d.end(), 0.0,
                    [](double a, double b){ return a + std::abs(b); }) / tr.d.size();
                double v_err = 0.0;  // optional

                // Total cost with proximity penalty
                tr.cost = p_.k_j * j_lat + p_.k_t * T + p_.k_d * dev + p_.k_v * v_err +
                          p_.k_proximity * proximity_cost;

                FRENET_LOG(LogLevel::DEBUG, "[Frenet] Trajectory (T=" << T << ", df=" << df
                          << "): NO collision, cost=" << tr.cost << " (proximity=" << proximity_cost
                          << ", checks=" << collision_checks << ")");

                cands.push_back(std::move(tr));
            } else if (tr.collision) {
                FRENET_LOG(LogLevel::DEBUG, "[Frenet] Trajectory (T=" << T << ", df=" << df
                          << ") REJECTED: collision detected after " << collision_checks << " checks");
            }
        }
    }

    return cands;
}





std::optional<FrenetTraj> FrenetPlanner::select_best(const std::vector<FrenetTraj>& cands)
{
    double best=1e18; size_t bi=std::numeric_limits<size_t>::max();
    for(size_t i=0;i<cands.size();++i){
        if(!cands[i].collision && cands[i].cost < best){ best=cands[i].cost; bi=i; }
    }
    if(bi==std::numeric_limits<size_t>::max()) return std::nullopt;
    return cands[bi];
}