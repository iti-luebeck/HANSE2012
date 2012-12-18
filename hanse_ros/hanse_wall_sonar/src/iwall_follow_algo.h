#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#define EIGEN_MATRIXBASE_PLUGIN "eigen_plugin/matrixbase_plugin.h"
#define EIGEN_QUATERNIONBASE_PLUGIN "eigen_plugin/quaternionbase_plugin.h"
#include <Eigen/Dense>
#include "geometry_msgs/PolygonStamped.h"

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
            const geometry_msgs::PolygonStamped::ConstPtr& msg,
            const geometry_msgs::Pose& pose,
            Vector3d &goal,
            Quaterniond &orientation) throw (std::runtime_error) = 0;
};

#endif // IWALL_FOLLOW_ALGO_H
