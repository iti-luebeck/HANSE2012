#include "ros/ros.h"
#include <ros/console.h>
#include <Eigen/Dense>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include <vector>

#ifndef GLOBAL_SONAR_H
#define GLOBAL_SONAR_H

using namespace Eigen;

//!
/*!

  */
class GlobalSonarNode {
public:
    //!
    /*!
      \param node handle of the node
     */
    GlobalSonarNode(ros::NodeHandle node);

    //! laser scan callback
    /*!

      \param msg received message.
     */
    void sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg);

    //! position callback
    /*!
      Callback method receiving position updates
      \param msg received message.
     */
    void pos_update(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    ros::Publisher pub;
    geometry_msgs::Pose last_pose;
    sensor_msgs::LaserScan last_laser_scan;
    ros::NodeHandle node;
};

/*!
  Start the global_sonar node. Isn't using any input arguments.
  \param argc unused
  \param argv unused
*/
int main(int argc, char **argv);

#endif // WALL_FOLLOW_H
