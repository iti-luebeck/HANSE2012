#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/OccupancyGrid.h>

class WorldMap {
public:
    WorldMap(const std::string &mapFile, float pixelSize, float threshold);
    float wallDistance(Eigen::Vector2f point) const;

    float directedWallDistance(Eigen::Vector2f point, Eigen::Vector2f direction, float maximum) const;

    Eigen::Vector2f mapSize() const;

    nav_msgs::OccupancyGrid const &occupancyGrid() const { return ogMap; }

private:
    float pixelSize;
    Eigen::ArrayXXf distanceMap;
    nav_msgs::OccupancyGrid ogMap;
};

#endif
