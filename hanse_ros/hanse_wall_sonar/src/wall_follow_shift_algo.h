#include "iwall_follow_algo.h"

#ifndef WALL_FOLLOW_SHIFT_ALGO_H
#define WALL_FOLLOW_SHIFT_ALGO_H

class wall_follow_shift_algo : public Iwall_follow_algo
{
public:
    void sonar_laser_update(
            const sensor_msgs::LaserScan::ConstPtr& msg,
            geometry_msgs::PoseStamped &goal);
};

#endif // WALL_FOLLOW_SHIFT_ALGO_H
