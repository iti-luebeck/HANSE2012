#include "ros/ros.h"
#include <ros/console.h>
#define EIGEN_MATRIXBASE_PLUGIN "eigen_plugin/matrixbase_plugin.h"
#define EIGEN_QUATERNIONBASE_PLUGIN "eigen_plugin/quaternionbase_plugin.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <exception>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "iwall_follow_algo.h"
#include "wall_follow_shift_algo.h"
#include "geometry_msgs/PolygonStamped.h"
#include <list>

#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

//! WallFollowNode
/*!
  Class that handles the logic for the WallFollow node.
  */
class WallFollowNode {
public:
    //! WallFollowNode Constructor
    /*!
      Constructor for WallFollowNode class
      \param n node handle of the node
     */
    WallFollowNode(ros::NodeHandle n);

    //! laser scan callback
    /*!
      Callback method receiving laser scan updates
      \param msg received message.
     */
    void sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg);

    //! laser position callback
    /*!
      Callback method receiving position updates
      \param msg received message.
     */
    void pos_update(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    ros::NodeHandle _n;
    ros::Publisher pub;
    ros::Publisher debug_pub;
    ros::Publisher debug_laser_pub;
    Iwall_follow_algo *algo;

   geometry_msgs::Pose last_pose;

   //! visualizing sonar in rviz
   /*!
     Rotate and relocate the laserscan to global coordinates,
     publishing a new polygon with the publisher debug_laser_pub
     \param msg the laserscan
     \param a transformation from robot to global coordinates
    */
   void transform_sonar_for_rviz(const sensor_msgs::LaserScan::ConstPtr& msg, Affine3d a);

};

/*!
  Start the wall following node. Isn't using any input arguments.
  \param argc unused
  \param argv unused
*/
int main(int argc, char **argv);

#endif // WALL_FOLLOW_H
