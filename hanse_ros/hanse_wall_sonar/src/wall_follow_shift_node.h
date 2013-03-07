#include <Eigen/Dense>
#include <exception>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
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
    /*!
     * \brief setup subscribers and publishers
     * pub_goal_ advertises to "sonar/wall_follow/goal", pub_poly_ advertises to "sonar/shift/poly" and call setupSubscribers()
     * \param n node handle
     */
    WallFollowShiftNode(ros::NodeHandle n);
    /*!
     * \brief callback of ros for sonar data
     * use shift algorithm to calculate a goal from current position and global sonar data.
     * \param msg global sonar data
     */
    void sonarLaserUpdate(const geometry_msgs::PolygonStamped::ConstPtr& msg) throw (std::runtime_error);
    /*!
     * \brief dynamic reconfigure callback
     * overwrite config
     * \param config new config
     * \param level
     */
    void configCallback(hanse_wall_sonar::wall_follow_shift_node_paramsConfig &config, uint32_t level);
    /*!
     * \brief callback of ros for new pose
     * store the current pose
     * \param msg current pose
     */
    void posUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
    ros::NodeHandle node_;
    ros::Publisher pub_poly_;
    ros::Publisher pub_goal_;


    ros::Subscriber sub_global_sonar_;
    ros::Subscriber sub_pos_;

    geometry_msgs::Pose last_pose_;

    hanse_wall_sonar::wall_follow_shift_node_paramsConfig config_;

    /*!
     * \brief setup the subscribers
     *  sub_laser_ subscribes to "sonar/global_sonar/polygon"
     *  sub_pos_ subscribes on "posemeter" (active simulation) or on "position/estimate" (real world)
     */
    void setupSubscribers();
    /*!
     * \brief publish shifted global sonar points
     * \param shifted_points
     */
    void publishDebugInfo(const std::vector<Vector3d> &shifted_points);


};

#endif // WALL_FOLLOW_SHIFT_NODE_H
