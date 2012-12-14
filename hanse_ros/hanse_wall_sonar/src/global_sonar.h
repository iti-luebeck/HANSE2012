#include "ros/ros.h"
#include <ros/console.h>
#include <Eigen/Dense>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include <vector>
#include "hanse_msgs/ELaserScan.h"
#include "pcl_ros/point_cloud.h"

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
    void sonar_laser_update(const hanse_msgs::ELaserScan::ConstPtr& msg);

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
    std::vector<geometry_msgs::Point32> last_points;
};

/*!
  Start the global_sonar node. Isn't using any input arguments.
  \param argc unused
  \param argv unused
*/
int main(int argc, char **argv);

#endif // WALL_FOLLOW_H
