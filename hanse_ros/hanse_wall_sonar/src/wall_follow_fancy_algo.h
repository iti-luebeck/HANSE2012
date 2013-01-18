#include "iwall_follow_algo.h"
#include <Eigen/Dense>
#include <exception>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include <float.h>
#include <list>
#include <set>
#include <vector>
#include "angles/angles.h"

#ifndef WALL_FOLLOW_FANCY_ALGO_H
#define WALL_FOLLOW_FANCY_ALGO_H

//enable debug mode
#define DEBUG
class WallFollowFancyAlgo : public Iwall_follow_algo
{
public:
    WallFollowFancyAlgo();
    void sonarLaserUpdate(
            const geometry_msgs::PolygonStamped::ConstPtr& msg,
            const geometry_msgs::Pose& pose,
            Vector3d &goal,
            Quaterniond &orientation) throw (std::runtime_error);


private:
    bool isBehindRobot(const Vector3d &p, const double &robot_yaw_angle, const Vector3d &robot_position);
    bool isInsideOtherCircle(const double &distance, const std::vector<Vector3d> &global_sonar_points, const Vector3d &pc, const double &tolerance);
#ifdef DEBUG
    ros::NodeHandle node_;
    ros::Publisher pub_all_;
    ros::Publisher pub_path_;
    void publishDebugInfo(const std::list<Vector3d> &all,const std::vector<Vector3d> &path);
#endif //DEBUG


};

#endif // WALL_FOLLOW_SHIFT_ALGO_H
