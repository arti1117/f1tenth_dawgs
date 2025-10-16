#ifndef F1TENTH_PLANNERS_OCCUPANCY_MAP_HPP
#define F1TENTH_PLANNERS_OCCUPANCY_MAP_HPP

#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>


namespace f1tenth {

    class OccupancyMap {
    public:
        OccupancyMap();
        void update(const nav_msgs::msg::OccupancyGrid &grid);
        // query whether circle at (x,y) with radius r collides
        bool collision(double x, double y, double r) const;
        // sample obstacles as point list (for frenet usage)
        std::vector<std::pair<double,double>> sample_obstacles(double resolution=0.1) const;
    private:
        nav_msgs::msg::OccupancyGrid grid_;
        bool has_ = false;
        // bool collisionRect(double x, double y, double yaw, double length, double width) const;
    };

}


#endif