#include "iwall_follow_algo.h"
#define EIGEN_MATRIX_PLUGIN "eigen_plugin/matrixbase_plugin.h"
#define EIGEN_QUATERNIONBASE_PLUGIN "eigen_plugin/quaternionbase_plugin.h"
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
class wall_follow_fancy_algo : public Iwall_follow_algo
{
public:
    wall_follow_fancy_algo();
    void sonar_laser_update(
            const geometry_msgs::PolygonStamped::ConstPtr& msg,
            const geometry_msgs::Pose& pose,
            Vector3d &goal,
            Quaterniond &orientation) throw (std::runtime_error);
    Vector3d test(std::vector<Vector3d> global_sonar_points, double distance, double tolerance, std::list<Vector3d>::iterator it, std::list<Vector3d> pCirc);

private:
    ros::NodeHandle n;
    ros::Publisher pub_all;
    ros::Publisher pub_path;

    bool is_behind_robot(const Vector3d &p, const double &robot_yaw_angle, const Vector3d &robot_position);
    bool is_inside_other_circle(const double &distance, const std::vector<Vector3d> &global_sonar_points, const Vector3d &pc, const double &tolerance);
#ifdef DEBUG
    void publish_debug_info(const std::list<Vector3d> &all,const std::vector<Vector3d> &path);
#endif //DEBUG


};

#endif // WALL_FOLLOW_SHIFT_ALGO_H