// pure_pursuit_node.cpp

#include "pure_pursuit/pure_pursuit_node.hpp"

using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode() : Node("pure_pursuit") {
    // Declare parameters
    this->declare_parameter<std::string>("waypoint_file", "waypoints.csv");
    this->declare_parameter<std::string>("csv_log_path", "lap_log.csv");
    this->declare_parameter<double>("lookahead_base", 1.0);
    this->declare_parameter<double>("lookahead_k", 0.4);
    this->declare_parameter<double>("wheelbase", 0.33);
    this->declare_parameter<double>("lateral_threshold", 1.0);
    this->declare_parameter<double>("v_min_lap", 0.5);
    this->declare_parameter<double>("min_lap_interval", 10.0);
    this->declare_parameter<int>("local_window_size", 40);
    this->declare_parameter<bool>("use_speed_lookahead", true);
    this->declare_parameter<bool>("rotate_path_on_start", true);

    // topics
    this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("lap_time_topic", "/lap_time");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "ego_racecar/base_link");

    wp_topic_ = "waypoints";
    str_topic_ = "steerings";
    wp_near_topic_ = "wp_near";
    wp_ahead_topic_ = "wp_ahead";

    // Get parameters
    waypoint_file_ = this->get_parameter("waypoint_file").as_string();
    csv_log_path_ = this->get_parameter("csv_log_path").as_string();
    lookahead_base_ = this->get_parameter("lookahead_base").as_double();
    lookahead_k_ = this->get_parameter("lookahead_k").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    lateral_threshold_ = this->get_parameter("lateral_threshold").as_double();
    v_min_lap_ = this->get_parameter("v_min_lap").as_double();
    min_lap_interval_ = this->get_parameter("min_lap_interval").as_double();
    local_window_ = this->get_parameter("local_window_size").as_int();
    use_speed_lookahead_ = this->get_parameter("use_speed_lookahead").as_bool();
    rotate_on_start_ = this->get_parameter("rotate_path_on_start").as_bool();

    // topics
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    lap_topic_ = this->get_parameter("lap_time_topic").as_string();

    // publishers/subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 50, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    lap_pub_ = this->create_publisher<std_msgs::msg::Float64>(lap_topic_, 10);

    wp_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(wp_topic_, 10);
    str_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(str_topic_, 10);
    wp_near_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(wp_near_topic_, 10);
    wp_ahead_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(wp_ahead_topic_, 10);

    // load waypoints
    if (!loadWaypointsCSV(waypoint_file_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load waypoint file: %s", waypoint_file_.c_str());
    } else {
        computeSlookup();
        buildKDTree();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints, track length=%.3f m",
                    cloud_.pts.size(), S_total_);
    }

    // CSV logging
    if (std::ifstream(csv_log_path_).good()){
        csv_out_.open(csv_log_path_, std::ios::app);
        if (csv_out_.is_open()) {
            csv_out_ << "timestamp,lap_time_s,s_at_cross,lateral_m\n";
            csv_out_.flush();
        }
    }

    last_lap_time_ = this->now();
    lap_start_time_ = this->now();
    prev_odom_time_ = rclcpp::Time(0,0);
    s_prev_initialized_ = false;
    rotated_already_ = false;

    RCLCPP_INFO(this->get_logger(), "pure_pursuit_node ready.");
}

  // ---------- load CSV ----------
bool PurePursuitNode::loadWaypointsCSV(const std::string &fname) {
    cloud_.pts.clear();
    std::ifstream ifs(fname);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open waypoint file: %s", fname.c_str());
        return false;
    }
    std::string line;
    std::getline(ifs, line);
    while (std::getline(ifs, line)) {
        if (line.size() == 0) continue;

        std::stringstream ss(line);
        std::string tok;
        std::vector<std::string> toks;
        while (std::getline(ss, tok, ',')) {
            // strip spaces
            tok.erase(0, tok.find_first_not_of(" \t\r\n"));
            tok.erase(tok.find_last_not_of(" \t\r\n") + 1);
            toks.push_back(tok);
        }

        Waypoint w;
        w.x = std::stod(toks[0]);
        w.y = std::stod(toks[1]);
        w.v = std::stod(toks[2]);

        cloud_.pts.push_back(w);
    }

    visualize_waypoint(cloud_.pts);

    ifs.close();
    return cloud_.pts.size() >= 2;
}

// ---------- build KD-tree ----------
void PurePursuitNode::buildKDTree() {
    if (kdtree_) { delete kdtree_; kdtree_ = nullptr; }
    kdtree_ = new KDTree(2, cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree_->buildIndex();
}

// ---------- s lookup ----------
void PurePursuitNode::computeSlookup() {
    s_lookup_.clear();
    if (cloud_.pts.empty()) return;
    s_lookup_.push_back(0.0);
    for (size_t i = 1; i < cloud_.pts.size(); ++i) {
        double dx = cloud_.pts[i].x - cloud_.pts[i-1].x;
        double dy = cloud_.pts[i].y - cloud_.pts[i-1].y;
        double seg = std::hypot(dx, dy);
        s_lookup_.push_back(s_lookup_.back() + seg);
    }
    S_total_ = s_lookup_.back();
}

// ---------- rotate path so nearest becomes first ----------
void PurePursuitNode::rotatePathToNearest(double px, double py) {
    if (!rotate_on_start_ || rotated_already_) return;
    if (cloud_.pts.empty()) return;

    // find nearest using KD-tree (buildKDTree must be called before)
    double q[2] = {px, py};
    size_t ret_idx;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_idx, &out_dist_sqr);
    if (kdtree_) {
        kdtree_->findNeighbors(resultSet, q, nanoflann::SearchParameters()); //was SearchParams
    } else {
        // fallback naive
        double best = std::numeric_limits<double>::infinity();
        for (size_t i=0;i<cloud_.pts.size();++i){
        double dx = cloud_.pts[i].x - px;
        double dy = cloud_.pts[i].y - py;
        double d2 = dx*dx + dy*dy;
        if (d2 < best){ best = d2; ret_idx = i; }
        }
    }
    size_t nearest = ret_idx;
    if (nearest > 0) {
        std::rotate(cloud_.pts.begin(), cloud_.pts.begin() + nearest, cloud_.pts.end());
        // recompute s_lookup and rebuild kdtree
        computeSlookup();
        buildKDTree();
        RCLCPP_INFO(this->get_logger(), "Rotated path so index %zu becomes start", nearest);
    } else {
        RCLCPP_INFO(this->get_logger(), "No rotation needed (nearest index 0)");
    }
    rotated_already_ = true;
}

double PurePursuitNode::curvature(const Waypoint& p1, const Waypoint& p2, const Waypoint& p3) {
    double denom = (p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y) + p3.x*(p1.y-p2.y));
    if (fabs(denom) < 1e-6) return 0.0; // straight
    double num = 2.0 * ((p1.x*p2.y + p2.x*p3.y + p3.x*p1.y) - (p1.y*p2.x + p2.y*p3.x + p3.y*p1.x));
    return num / denom;
}

double PurePursuitNode::adjustSpeed(const std::vector<Waypoint> & wps, size_t idx, LookaheadResult & la) {
    // curvature based speed
    size_t prev = (idx + wps.size() - 1) % wps.size();
    size_t next = (idx + 1) % wps.size();

    double kappa = curvature(wps[prev], wps[idx], wps[next]);

    double mu = 0.9; // friction coeff
    double g = 9.81;
    double v_curve = sqrt(mu * g / (fabs(kappa) + 1e-6));

    return std::min(std::min(wps[idx].v, v_curve), la.v);
}


// ---------- projection (local window) ----------
Projection PurePursuitNode::projectToPathLocal(double px, double py, int nearest_idx) {
    Projection res;
    res.valid = false;
    res.s = 0.0;
    res.lateral = 1e9;
    res.seg_idx = nearest_idx;

    if (cloud_.pts.size() < 2) return res;

    int N = static_cast<int>(cloud_.pts.size());
    int start = std::max(0, nearest_idx - local_window_);
    int end = std::min(N - 2, nearest_idx + local_window_);

    double best_d2 = std::numeric_limits<double>::infinity();
    double best_s = 0.0;
    int best_i = nearest_idx;

    for (int i = start; i <= end; ++i) {
        const auto &A = cloud_.pts[i];
        const auto &B = cloud_.pts[i+1];
        double vx = B.x - A.x;
        double vy = B.y - A.y;
        double wx = px - A.x;
        double wy = py - A.y;
        double vlen2 = vx*vx + vy*vy;
        double t = 0.0;
        if (vlen2 > 1e-9) {
        t = (wx*vx + wy*vy) / vlen2;
        t = std::max(0.0, std::min(1.0, t));
        }
        double projx = A.x + t * vx;
        double projy = A.y + t * vy;
        double d2 = (px - projx)*(px - projx) + (py - projy)*(py - projy);
        if (d2 < best_d2) {
        best_d2 = d2;
        double seglen = std::sqrt(vlen2);
        double s_at_proj = s_lookup_[i] + t * seglen;
        best_s = s_at_proj;
        best_i = i;
        }
    }

// fallback: full search if nothing found
    if (best_d2 == std::numeric_limits<double>::infinity()) {
        for (int i = 0; i < N-1; ++i) {
        const auto &A = cloud_.pts[i];
        const auto &B = cloud_.pts[i+1];
        double vx = B.x - A.x;
        double vy = B.y - A.y;
        double wx = px - A.x;
        double wy = py - A.y;
        double vlen2 = vx*vx + vy*vy;
        double t = 0.0;
        if (vlen2 > 1e-9) {
            t = (wx*vx + wy*vy) / vlen2;
            t = std::max(0.0, std::min(1.0, t));
        }
        double projx = A.x + t * vx;
        double projy = A.y + t * vy;
        double d2 = (px - projx)*(px - projx) + (py - projy)*(py - projy);
        if (d2 < best_d2) {
            best_d2 = d2;
            double seglen = std::sqrt(vlen2);
            double s_at_proj = s_lookup_[i] + t * seglen;
            best_s = s_at_proj;
            best_i = i;
        }
        }
    }

    res.s = best_s;
    res.lateral = std::sqrt(best_d2);
    res.seg_idx = best_i;
    res.valid = (res.lateral <= lateral_threshold_);
    return res;
}

int PurePursuitNode::determineDirection(size_t idx, double yaw) {
    size_t next_idx = (idx + 1) % cloud_.pts.size();
    size_t prev_idx = (idx + cloud_.pts.size() - 1) % cloud_.pts.size();

    // track 진행방향: idx -> next_idx
    double dx = cloud_.pts[next_idx].x - cloud_.pts[idx].x;
    double dy = cloud_.pts[next_idx].y - cloud_.pts[idx].y;

    // Vehicle heading
    double hx = cos(yaw);
    double hy = sin(yaw);

    double dot = dx * hx + dy * hy;

    if (dot >= 0) {
        return +1; // 정방향
    } else {
        return -1; // 역방향
    }
}

// ---------- find lookahead point with exact segment interpolation ----------
// start_idx = index of nearest waypoint
// returns interpolated (x,y) and interpolated v and segment index
LookaheadResult PurePursuitNode::findLookaheadPointInterpolated(int start_idx, double lookahead_dist, int direction) {
    LookaheadResult res;
    int N = static_cast<int>(cloud_.pts.size());
    if (N < 2) {
        res.x = res.y = 0.0; res.v = 0.0; res.idx = 0; res.seg_t = 0.0;
        return res;
    }

    // We will walk segments starting from start_idx until we accumulate >= lookahead_dist
    double accum = 0.0;
    int idx = start_idx;
    // If start_idx refers to a waypoint, we consider segment idx->idx+1 as first segment
    int safety = 0;
    while (accum < lookahead_dist && safety < N*2) {
        size_t next = (direction > 0) ? (idx + 1) % N : (idx + N-1) % N;
        // int next = (idx + 1) % N;
        double dx = cloud_.pts[next].x - cloud_.pts[idx].x;
        double dy = cloud_.pts[next].y - cloud_.pts[idx].y;
        double seg = std::hypot(dx, dy);
        if (accum + seg >= lookahead_dist) {
        // target lies within this segment
        double remain = lookahead_dist - accum;
        double t = (seg < 1e-9) ? 0.0 : (remain / seg);
        // linear interpolate position and speed
        res.x = cloud_.pts[idx].x + t * dx;
        res.y = cloud_.pts[idx].y + t * dy;
        res.v = cloud_.pts[idx].v + t * (cloud_.pts[next].v - cloud_.pts[idx].v);
        res.idx = next;
        res.seg_t = t;
        return res;
        }
        accum += seg;
        idx = next;
        safety++;
    }
    // fallback: return last waypoint
    res.x = cloud_.pts.back().x;
    res.y = cloud_.pts.back().y;
    res.v = cloud_.pts.back().v;
    res.idx = static_cast<int>(cloud_.pts.size()) - 1;
    res.seg_t = 0.0;
    return res;
}

// ---------- odom callback ----------
void PurePursuitNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (cloud_.pts.size() < 2) return;

    double px = msg->pose.pose.position.x;
    double py = msg->pose.pose.position.y;
    double yaw = quatToYaw(msg->pose.pose.orientation);
    double vx = msg->twist.twist.linear.x;
    rclcpp::Time now = this->now();

    // rotate path first time if requested
    if (rotate_on_start_ && !rotated_already_) {
        // ensure kd-tree exists
        if (!kdtree_) buildKDTree();
        rotatePathToNearest(px, py);
        // after rotation, recompute lookups done inside rotatePathToNearest
    }

    // NN search via KD-tree
    double query[2] = {px, py};
    size_t ret_index = 0;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);
    kdtree_->findNeighbors(resultSet, query, nanoflann::SearchParameters()); //SearchParams
    int nearest_idx = static_cast<int>(ret_index);


    // accurate projection around nearest
    auto proj = projectToPathLocal(px, py, nearest_idx);
    double s_cur = proj.s;
    double lateral = proj.lateral;

    visualize_nearest_wp(cloud_.pts, nearest_idx);

    // initialize prev values
    if (!s_prev_initialized_) {
        s_prev_ = s_cur;
        prev_s_ = s_cur;
        prev_px_ = px; prev_py_ = py;
        prev_odom_time_ = now;
        lap_start_time_ = now;
        s_prev_initialized_ = true;
    }

    // compute lookahead distance
    double Ld = lookahead_base_;
    if (use_speed_lookahead_) Ld = lookahead_base_ + lookahead_k_ * std::max(0.0, vx);
    if (Ld < 0.05) Ld = 0.05;

    int direction = determineDirection(nearest_idx, yaw);
    // find lookahead (interpolated)
    auto la = findLookaheadPointInterpolated(nearest_idx, Ld, direction);

    visualize_lookahead_wp(cloud_.pts, la.x, la.y);

    // compute in vehicle frame
    double dx = la.x - px;
    double dy = la.y - py;
    double x_bl =  std::cos(-yaw)*dx - std::sin(-yaw)*dy;
    double y_bl =  std::sin(-yaw)*dx + std::cos(-yaw)*dy;

    // pure pursuit steering
    double alpha = std::atan2(y_bl, std::max(1e-9, x_bl));
    double delta = std::atan2(2.0 * wheelbase_ * std::sin(alpha), std::max(1e-9, Ld));

    // prepare drive message
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = now;
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle = delta;
    // set speed to lookahead-interpolated desired speed
    drive_msg.drive.speed = adjustSpeed(cloud_.pts, la.idx, la);
    drive_pub_->publish(drive_msg);

    visualize_steering(drive_msg.drive.speed, drive_msg.drive.steering_angle);

    // ---------- lap detection with crossing time interpolation ----------
    double wrap_thresh = 0.1 * S_total_;
    bool wrapped = false;
    if ((s_prev_ > (S_total_ - wrap_thresh) && s_cur < wrap_thresh) ||
        ((s_cur - s_prev_) < - (S_total_ * 0.5))) {
        wrapped = true;
    }
    bool in_start_gate = (s_cur < wrap_thresh || s_cur > (S_total_ - wrap_thresh));
    bool speed_ok = (vx >= v_min_lap_);
    bool lateral_ok = (lateral <= lateral_threshold_);
    bool time_ok = ((now - last_lap_time_).seconds() > min_lap_interval_);

    if ((wrapped || in_start_gate) && speed_ok && lateral_ok && time_ok) {
        // compute crossing time by linear interpolation between prev and now
        double lap_time_s = 0.0;
        rclcpp::Time t_cross = now;
        if (wrapped) {
        // total travel between prev_s and s_cur across wrap:
        double dist_to_end = S_total_ - prev_s_;
        double dist_after = s_cur;
        double total_travel = dist_to_end + dist_after;
        double frac = 0.0;
        if (total_travel > 1e-9) frac = dist_to_end / total_travel;
        // clamp
        frac = std::max(0.0, std::min(1.0, frac));
        double dt = (now - prev_odom_time_).seconds();
        double dt_cross = frac * dt;
        t_cross = prev_odom_time_ + rclcpp::Duration::from_seconds(dt_cross);
        lap_time_s = (t_cross - lap_start_time_).seconds();
        } else {
        // inside gate case without wrap; we can interpolate to nearest boundary (e.g., s=0)
        // simpler: linear interpolation proportion from prev_s to s_cur to crossing at s=0 (if prev >0 and cur near 0)
        double total_travel = std::abs(s_cur - prev_s_);
        double frac = 0.0;
        if (total_travel > 1e-9) {
            // if prev_s > s_cur (decrease) but not wrapped, consider crossing when s==0 approx -> not common
            frac = (prev_s_) / (prev_s_ - s_cur); // heuristic
            frac = std::max(0.0, std::min(1.0, frac));
        }
        double dt = (now - prev_odom_time_).seconds();
        double dt_cross = frac * dt;
        t_cross = prev_odom_time_ + rclcpp::Duration::from_seconds(dt_cross);
        lap_time_s = (t_cross - lap_start_time_).seconds();
        }

        // publish and log
        std_msgs::msg::Float64 lm;
        lm.data = lap_time_s;
        lap_pub_->publish(lm);
        if (csv_out_.is_open()) {
        csv_out_ << t_cross.seconds() << "," << lap_time_s << "," << s_cur << "," << lateral << "\n";
        csv_out_.flush();
        }
        RCLCPP_INFO(this->get_logger(), "Lap completed (corrected): %.3f s  (s=%.3f lat=%.3f v=%.3f)",
                    lap_time_s, s_cur, lateral, vx);

        // update lap timers
        last_lap_time_ = t_cross;
        lap_start_time_ = t_cross;
        // update prev and return to avoid double counting on same callback
        prev_odom_time_ = now;
        prev_s_ = s_cur;
        s_prev_ = s_cur;
        return;
    }

    // update previous states for next cycle
    prev_odom_time_ = now;
    prev_px_ = px; prev_py_ = py;
    prev_s_ = s_cur;
    s_prev_ = s_cur;
    last_lap_time_ = last_lap_time_; // unchanged
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
