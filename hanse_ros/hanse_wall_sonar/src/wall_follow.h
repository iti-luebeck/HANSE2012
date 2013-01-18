#include "ros/ros.h"
#include <ros/console.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <exception>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "iwall_follow_algo.h"
#include "wall_follow_shift_algo.h"
#include "wall_follow_fancy_algo.h"
#include "geometry_msgs/PolygonStamped.h"
#include <list>

#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#define DEBUG
#define SIMULATION_MODE

//! WallFollowNode
/*!
  Class that handles the logic for the WallFollow node.
  */
class WallFollowNode {
public:
    //! WallFollowNode Constructor
    /*!
      Constructor for WallFollowNode class
      \param node handle of the node
     */
    WallFollowNode(ros::NodeHandle n);

    //! laser scan callback
    /*!
      Callback method receiving laser scan updates
      \param msg received message.
     */
    void gSonarUpdate(const geometry_msgs::PolygonStamped::ConstPtr& msg);

    //! position callback
    /*!
      Callback method receiving position updates
      \param msg received message.
     */
    void posUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    ros::NodeHandle node_;
    ros::Publisher pub_;
    ros::Publisher debug_pub_;
    ros::Publisher debug_laser_pub_;
    Iwall_follow_algo *algo_;

    geometry_msgs::Pose last_pose_;

    int32_t last_goal_update_;

};

/*!
  Start the wall following node. Isn't using any input arguments.
  \param argc unused
  \param argv unused
*/
int main(int argc, char **argv);

#endif // WALL_FOLLOW_H
