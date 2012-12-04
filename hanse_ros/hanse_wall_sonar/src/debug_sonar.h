#include "ros/ros.h"
#include <ros/console.h>
#include <Eigen/Dense>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include <vector>

#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

using namespace Eigen;

//! WallFollowNode
/*!
  Class that handles the logic for the debug_sonar node.
  */
class DebugSonarNode {
public:
    //! WallFollowNode Constructor
    /*!
      Constructor for WallFollowNode class
      \param n node handle of the node
     */
    DebugSonarNode(ros::NodeHandle node);

    //! laser scan callback
    /*!
      Callback method receiving laser scan updates, publishes
      rotated and relocated laserscan in global coordinates, as Marker.
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
    ros::Publisher pub;
    geometry_msgs::Pose last_pose;
    ros::NodeHandle node;
};

/*!
  Start the debug_sonar node. Isn't using any input arguments.
  \param argc unused
  \param argv unused
*/
int main(int argc, char **argv);

#endif // WALL_FOLLOW_H
