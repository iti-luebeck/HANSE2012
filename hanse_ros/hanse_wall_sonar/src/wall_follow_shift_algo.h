#include "iwall_follow_algo.h"

#include <set>
#define EIGEN_MATRIXBASE_PLUGIN "eigen_plugin.h"
#include <Eigen/Dense>

#ifndef WALL_FOLLOW_SHIFT_ALGO_H
#define WALL_FOLLOW_SHIFT_ALGO_H

class wall_follow_shift_algo : public Iwall_follow_algo
{
public:
    void sonar_laser_update(
            const sensor_msgs::LaserScan::ConstPtr& msg,
            Eigen::Vector2d &goal);
};

#endif // WALL_FOLLOW_SHIFT_ALGO_H
