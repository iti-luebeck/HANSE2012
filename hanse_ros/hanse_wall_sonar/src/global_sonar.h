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
#include <list>
#include <math.h>

#ifndef ORIENTATION_AWARE_GLOBAL_SONAR_H
#define ORIENTATION_AWARE_GLOBAL_SONAR_H

//#define SIMULATION_MODE
#ifdef SIMULATION_MODE
    #include "hanse_msgs/ELaserScan.h"
#else
    #include "hanse_msgs/WallDetection.h"
#endif

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
    GlobalSonarNode(ros::NodeHandle node_);


#ifdef SIMULATION_MODE
    //! laser scan callback
    /*!
      \param msg received message.
     */
    void sonarLaserUpdate(const hanse_msgs::ELaserScan::ConstPtr& msg);

#else
    //! WallDetecion callback
    /*!
      \param msg received message.
     */
    void wallsUpdate(const hanse_msgs::WallDetection::ConstPtr& msg);
#endif

    //! position callback
    /*!
      Callback method receiving position updates
      \param msg received message.
     */
    void posUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    //! publisher for the global sonar
    ros::Publisher pub_;
    //! handle of global sonar node
    ros::NodeHandle node_;

#ifdef SIMULATION_MODE
    //! last global sonar points
    std::vector<geometry_msgs::Point32> last_points_;
    //! list of which sonar points are valid (avoiding vector<bool>)
    std::vector<char> last_valid_points_;

    //! last newchange from e_laser_scan message
    uint16_t last_newchange_;
    //! index of the last changed value of internal laser scan
    uint16_t last_j_;
#else
    //! time to store WallDetection positions.
    uint32_t store_time_sec_;
    //! datastructure to store incomming WallDetection messages.
    struct posStamped{
        geometry_msgs::Point32 pos_;
        uint32_t sec_;
    };
    //! list of all messages that are stored.
    std::list<posStamped> pos_list_;
#endif

    //! last known pose
    geometry_msgs::Pose last_pose_;


    //! returns the Affine3d to transform from robot to global coordinates
    Affine3d getRobotTransform();
    //! returns the z rotation of the robot
    /*!
     * returns the z rotation of the robot
     * \return z rotation of the robot. Between 0 and 2pi.
     */
    double getRobotZRot();

    /*!
     * \param local_angle angle of sonar refering to robot coordinate system.
     * \param local_distance distance to robot.
     * \return point in global coordinate system.
     */
    geometry_msgs::Point32 calculateGlobalPoint(double local_angle, double local_distance);
};

/*!
  Start the orientation_aware_global_sonar node. Isn't using any input arguments.
  \param argc unused
  \param argv unused
*/
int main(int argc, char **argv);


#endif // ORIENTATION_AWARE_GLOBAL_SONAR_H
