#include "ros/ros.h"
#include <ros/console.h>
#include <Eigen/Dense>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "visualization_msgs/Marker.h"
#include <vector>
#include "hanse_msgs/ELaserScan.h"
#include <math.h>

#ifndef ORIENTATION_AWARE_GLOBAL_SONAR_H
#define ORIENTATION_AWARE_GLOBAL_SONAR_H

using namespace Eigen;

//!
/*!
 * Calculates orientation aware global sonar data.
 * Attention! The order isn't perfect for translational moving.
 * (We only compansate the orientation of the sonar)
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
    //! publisher for the global sonar
    ros::Publisher pub;
    //! handle of global sonar node
    ros::NodeHandle node;

    //! last global sonar points
    std::vector<geometry_msgs::Point32> last_points;
    //! list of which sonar points are valid (avoiding vector<bool>)
    std::vector<char> last_valid_points;

    //! last known pose
    geometry_msgs::Pose last_pose;

    //! last newchange from e_laser_scan message
    uint16_t last_newchange;
    //! index of the last changed value of internal laser scan
    uint16_t last_j;

    //! returns the Affine3d to transform from robot to global coordinates
    Affine3d get_robot_transform();
    //! returns the z rotation of the robot
    /*!
     * returns the z rotation of the robot
     * \return z rotation of the robot. Between 0 and 2pi.
     */
    double get_robot_z_rot();
};

/*!
  Start the orientation_aware_global_sonar node. Isn't using any input arguments.
  \param argc unused
  \param argv unused
*/
int main(int argc, char **argv);


#endif // ORIENTATION_AWARE_GLOBAL_SONAR_H
