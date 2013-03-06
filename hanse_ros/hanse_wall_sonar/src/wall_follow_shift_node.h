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

using namespace Eigen;

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
#include <hanse_wall_sonar/wall_follow_shift_node_paramsConfig.h>

#ifndef WALL_FOLLOW_SHIFT_NODE_H
#define WALL_FOLLOW_SHIFT_NODE_H

class WallFollowShiftNode
{
public:
    WallFollowShiftNode(ros::NodeHandle n);
    void sonarLaserUpdate(const geometry_msgs::PolygonStamped::ConstPtr& msg) throw (std::runtime_error);
    void configCallback(hanse_wall_sonar::wall_follow_shift_node_paramsConfig &config, uint32_t level);

    void posUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
    ros::NodeHandle node_;
    ros::Publisher pub_poly_;
    ros::Publisher pub_goal_;


    ros::Subscriber sub_laser_;
    ros::Subscriber sub_pos_;

    geometry_msgs::Pose last_pose_;

    hanse_wall_sonar::wall_follow_shift_node_paramsConfig config_;

    void setupSubscribers();
    void publishDebugInfo(const std::vector<Vector3d> &shifted_points);


};

#endif // WALL_FOLLOW_SHIFT_NODE_H
