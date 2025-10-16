#include "path_planner/frenet.hpp"
#include <limits>
#include <algorithm>


using namespace f1tenth;

static size_t nearest_index(const std::vector<Waypoint>& ref, double x, double y)
{
    size_t best=0;
    double bd=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<ref.size();++i){
        double d=distance(x,y,ref[i].x,ref[i].y);
        if(d<bd){bd=d; best=i;}
    }
    return best;
}

bool FrenetPlanner::cart2frenet(double x, double y, size_t &idx, FrenetState &out) const
{
    if(ref_.size()<2) return false;
    idx = nearest_index(ref_,x,y);
    size_t i2 = std::min(idx+1, ref_.size()-1);
    double rx=ref_[idx].x, ry=ref_[idx].y;
    double tx=ref_[i2].x-rx, ty=ref_[i2].y-ry;
    double nx=-std::sin(ref_[idx].yaw), ny=std::cos(ref_[idx].yaw);
    double dx=x-rx, dy=y-ry;
    double s=ref_[idx].s + (dx*tx+dy*ty)/std::max(1e-6,std::hypot(tx,ty));
    double d=dx*nx+dy*ny;
    out = {s,d,0,0,0};
    return true;
}


bool FrenetPlanner::frenet2cart(double s, double d, double &x, double &y, double &yaw) const
{
    if(ref_.size()<2) return false;
    // find segment containing s
    size_t i=0;
    while(i+1<ref_.size() && ref_[i+1].s < s) ++i;
    if(i+1>=ref_.size()) i=ref_.size()-2;
    const auto &p0 = ref_[i];
    const auto &p1 = ref_[i+1];
    const double ds = std::max(1e-6, p1.s - p0.s);
    const double r = (s - p0.s) / ds;
    const double cx = p0.x + r*(p1.x - p0.x);
    const double cy = p0.y + r*(p1.y - p0.y);
    const double cyaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    const double nx = -std::sin(cyaw), ny = std::cos(cyaw);
    x = cx + nx * d;
    y = cy + ny * d;
    yaw = cyaw;
    return true;
}

std::vector<FrenetTraj> FrenetPlanner::generate(const FrenetState& fs, const std::vector<std::pair<double,double>>& obstacles)
{
    std::vector<FrenetTraj> cands;
    if(ref_.empty()) return cands;


    // For each lateral goal d_f and terminal time T, use quintic poly for d(t), quartic for s(t)
    for(double T : p_.t_samples){
        for(double df : p_.d_samples){
            // lateral quintic: boundary (d, d', d'') -> (df, 0, 0)
            Eigen::Matrix<double,3,3> A;
            A << pow(T,3), pow(T,4), pow(T,5),
            3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
            6*T, 12*pow(T,2), 20*pow(T,3);
            Eigen::Vector3d b; b << df - (fs.d + fs.ds*T + 0.5*fs.dd*T*T),
            - (fs.ds + fs.dd*T),
            - fs.dd;
            Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
            double a0=fs.d, a1=fs.ds, a2=0.5*fs.dd, a3=x(0), a4=x(1), a5=x(2);


            // longitudinal quartic: boundary (s, s', s'') -> (s + v*T, v, 0)
            double v = p_.target_speed;
            Eigen::Matrix<double,2,2> As;
            As << 3*pow(T,2), 4*pow(T,3),
            6*T, 12*pow(T,2);
            Eigen::Vector2d bs; bs << (v - (fs.ds + fs.dd*T)),
            - fs.dd;
            Eigen::Vector2d xs = As.colPivHouseholderQr().solve(bs);
            double b0=fs.s, b1=fs.ds, b2=0.5*fs.dd, b3=xs(0), b4=xs(1);


            FrenetTraj tr;
            for(double t=0.0; t<=T+1e-6; t+=p_.dt){
                tr.t.push_back(t);
                // s(t)
                double s = b0 + b1*t + b2*t*t + b3*t*t*t + b4*t*t*t*t;
                double ds = b1 + 2*b2*t + 3*b3*t*t + 4*b4*t*t*t;
                // d(t)
                double d = a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
                double dd = a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
                // curvature & limits (approx)
                if(std::abs(dd) > p_.max_accel*2) { tr.cost = 1e9; break; }
                if(ds < 0.0 || ds > p_.max_speed*1.5) { tr.cost = 1e9; break; }
                double x,y,yaw; if(!frenet2cart(s,d,x,y,yaw)){ tr.cost=1e9; break; }
                tr.s.push_back(s); tr.d.push_back(d);
                tr.x.push_back(x); tr.y.push_back(y); tr.yaw.push_back(yaw);
            }


            // Collision check (circle buffer)
            if(tr.x.size()>1){
            for(size_t i=0;i<tr.x.size();++i){
                for(const auto &ob: obstacles){
                    if(distance(tr.x[i], tr.y[i], ob.first, ob.second) < p_.safety_radius){ tr.collision=true; break; }
                }
                if(std::abs(tr.d[i]) > p_.road_half_width) { tr.collision=true; break; }
                if(tr.collision) break;
                }
            }


            if(!tr.collision && tr.x.size()>1){
                // Simple cost: lateral jerk proxy + time + lateral deviation + speed error
                double j_lat=0.0;
                for(size_t i=2;i<tr.d.size();++i){ j_lat += std::abs(tr.d[i]-2*tr.d[i-1]+tr.d[i-2]); }
                double dev = std::accumulate(tr.d.begin(), tr.d.end(), 0.0, [](double a,double b){return a+std::abs(b);} )/tr.d.size();
                double v_err=0.0; // omitted for brevity
                tr.cost = p_.k_j*j_lat + p_.k_t*T + p_.k_d*dev + p_.k_v*v_err;
                cands.push_back(std::move(tr));
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