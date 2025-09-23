#include "path_planner/occupancy_map.hpp"
#include <cmath>

using namespace f1tenth;

OccupancyMap::OccupancyMap() {}

void OccupancyMap::update(const nav_msgs::msg::OccupancyGrid &grid) {
    grid_ = grid; has_ = true;
}


bool OccupancyMap::collision(double x, double y, double r) const {
    if(!has_) return false;
    double ox = grid_.info.origin.position.x;
    double oy = grid_.info.origin.position.y;
    double res = grid_.info.resolution;
    int w = grid_.info.width, h = grid_.info.height;
    int cx = static_cast<int>(std::floor((x - ox)/res));
    int cy = static_cast<int>(std::floor((y - oy)/res));
    int radius_cells = static_cast<int>(std::ceil(r / res));
    for(int dy = -radius_cells; dy <= radius_cells; ++dy){
        for(int dx = -radius_cells; dx <= radius_cells; ++dx){
            int nx = cx + dx; int ny = cy + dy;
            if(nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            int idx = ny * w + nx;
            int8_t val = grid_.data[idx];
            if(val > 50) return true; // occupied
        }
    }
    return false;
}

std::vector<std::pair<double,double>> OccupancyMap::sample_obstacles(double resolution) const {
    std::vector<std::pair<double,double>> out;
    if(!has_) return out;
    double ox = grid_.info.origin.position.x;
    double oy = grid_.info.origin.position.y;
    double res = grid_.info.resolution;
    int w = grid_.info.width, h = grid_.info.height;
    for(int y=0;y<h;y+=std::max(1, static_cast<int>(std::round(resolution/res)))){
        for(int x=0;x<w;x+=std::max(1, static_cast<int>(std::round(resolution/res)))){
            int idx = y*w + x;
            if(idx < 0 || idx >= (int)grid_.data.size()) continue;
            if(grid_.data[idx] > 50){
            double wx = ox + (x + 0.5) * res;
            double wy = oy + (y + 0.5) * res;
            out.emplace_back(wx, wy);
            }
        }
    }
    return out;
}