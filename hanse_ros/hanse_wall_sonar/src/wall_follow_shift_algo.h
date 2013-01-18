#include "iwall_follow_algo.h"
#include <Eigen/Dense>
#include <exception>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include <set>
#include <float.h>

#ifndef WALL_FOLLOW_SHIFT_ALGO_H
#define WALL_FOLLOW_SHIFT_ALGO_H

//enable debug mode
#define DEBUG
class WallFollowShiftAlgo : public Iwall_follow_algo
{
public:
    WallFollowShiftAlgo();
    void sonarLaserUpdate(
            const geometry_msgs::PolygonStamped::ConstPtr& msg,
            const geometry_msgs::Pose& pose,
            Vector3d &goal,
            Quaterniond &orientation) throw (std::runtime_error);
private:
    ros::NodeHandle node_;
    ros::Publisher pub_;
#ifdef DEBUG
    void publishDebugInfo(const std::vector<Vector3d> &shifted_points);
#endif //DEBUG


};

#endif // WALL_FOLLOW_SHIFT_ALGO_H
