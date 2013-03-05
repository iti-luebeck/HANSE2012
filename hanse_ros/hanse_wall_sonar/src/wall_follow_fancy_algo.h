#include "ros/ros.h"
#include <Eigen/Dense>
#include <exception>
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

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
#include <hanse_wall_sonar/wall_follow_fancy_algo_paramsConfig.h>

using namespace Eigen;

#ifndef WALL_FOLLOW_FANCY_ALGO_H
#define WALL_FOLLOW_FANCY_ALGO_H

class WallFollowFancyAlgoNode
{
public:
    WallFollowFancyAlgoNode(ros::NodeHandle n);
    void sonarLaserUpdate(const geometry_msgs::PolygonStamped::ConstPtr& msg) throw (std::runtime_error);


    void configCallback(hanse_wall_sonar::wall_follow_fancy_algo_paramsConfig &config, uint32_t level);

private:
    bool isBehindRobot(const Vector3d &p, const double &robot_yaw_angle, const Vector3d &robot_position);
    bool isInsideOtherCircle(const double &distance, const std::vector<Vector3d> &global_sonar_points, const Vector3d &pc);

    uint32_t last_goal_update_;

    geometry_msgs::Pose last_pose_;

    hanse_wall_sonar::wall_follow_fancy_algo_paramsConfig config_;

    ros::NodeHandle node_;
    ros::Publisher pub_all_;
    ros::Publisher pub_;
    ros::Publisher pub_path_;

    ros::Subscriber sub_laser_;
    ros::Subscriber sub_pos_;
    void publishDebugInfo(const std::list<Vector3d> &all,const std::vector<Vector3d> &path);

    void posUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void setupSubscribers();

};

//int main(int argc, char **argv);

#endif // WALL_FOLLOW_SHIFT_ALGO_H
