#include "iwall_follow_algo.h"
#define EIGEN_MATRIX_PLUGIN "eigen_plugin/matrixbase_plugin.h"
#define EIGEN_QUATERNIONBASE_PLUGIN "eigen_plugin/quaternionbase_plugin.h"
#include <Eigen/Dense>
#include <exception>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include <set>
#include <float.h>

#ifndef WALL_FOLLOW_SHIFT_ALGO_H
#define WALL_FOLLOW_SHIFT_ALGO_H

class wall_follow_shift_algo : public Iwall_follow_algo
{
public:
    void sonar_laser_update(
                const sensor_msgs::LaserScan::ConstPtr& msg,
                Vector3d &goal,
                Quaterniond &orientation) throw (std::runtime_error);
};

#endif // WALL_FOLLOW_SHIFT_ALGO_H
