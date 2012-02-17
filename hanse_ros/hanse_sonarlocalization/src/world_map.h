#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

class WorldMap {
public:
    WorldMap(const std::string &mapFile, float resolution);
    float wallDistance(Eigen::Vector2f point);

    Eigen::Vector2f mapSize();
private:
    float resolution;
    Eigen::ArrayXXf distanceMap;
};

#endif
