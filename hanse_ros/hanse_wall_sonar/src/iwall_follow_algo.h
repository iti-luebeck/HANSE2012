#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#define EIGEN_MATRIXBASE_PLUGIN "eigen_plugin/matrixbase_plugin.h"
#define EIGEN_QUATERNIONBASE_PLUGIN "eigen_plugin/quaternionbase_plugin.h"
#include <Eigen/Dense>

using namespace Eigen;

#ifndef IWALL_FOLLOW_ALGO_H
#define IWALL_FOLLOW_ALGO_H

class Iwall_follow_algo
{
public:
    /*!
      Methode is called if there is a laser scan updat.

      \param msg received message.
      \return coordinates of the goal relatively to HANSE
     */
    virtual void sonar_laser_update(
            const sensor_msgs::LaserScan::ConstPtr& msg,
            Eigen::Vector2d &goal) throw (std::runtime_error) = 0;
};

#endif // IWALL_FOLLOW_ALGO_H
