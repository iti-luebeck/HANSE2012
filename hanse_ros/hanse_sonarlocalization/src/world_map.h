#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

class WorldMap {
public:
    WorldMap(const std::string &mapFile, float pixelSize, float threshold);
    float wallDistance(Eigen::Vector2f point);

    float directedWallDistance(Eigen::Vector2f point, Eigen::Vector2f direction, float maximum);

    Eigen::Vector2f mapSize();
private:
    float pixelSize;
    Eigen::ArrayXXf distanceMap;
};

#endif
