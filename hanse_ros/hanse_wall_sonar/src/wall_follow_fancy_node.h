#include "ros/ros.h"
#include <Eigen/Dense>
#include <exception>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include <float.h>
#include <list>
#include <vector>
#include "angles/angles.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
#include <hanse_wall_sonar/wall_follow_fancy_node_paramsConfig.h>

using namespace Eigen;

#ifndef WALL_FOLLOW_FANCY_NODE_H
#define WALL_FOLLOW_FANCY_NODE_H

class WallFollowFancyNode
{
public:
    /*!
     * \brief Setup subscribers and publishers
     * advertising to "sonar/wall_follow/goal", "sonar/fancy/circles", "sonar/fancy/path" and call setupSubscribers()
     * \param n node handle
     */
    WallFollowFancyNode(ros::NodeHandle n);
    /*!
     * \brief callback of ros for sonar data
     * use fancy algorithm to calculate a goal from current position and global sonar data.
     * \param msg global sonar data
     */
    void sonarLaserUpdate(const geometry_msgs::PolygonStamped::ConstPtr& msg) throw (std::runtime_error);
    /*!
     * \brief callback of ros for new pose
     * store the current pose
     * \param msg current pose
     */
    void posUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg);
    /*!
     * \brief dynamic reconfigure callback
     * overwrite config
     * \param config new config
     * \param level
     */
    void configCallback(hanse_wall_sonar::wall_follow_fancy_node_paramsConfig &config, uint32_t level);

private:
    geometry_msgs::Pose last_pose_;

    hanse_wall_sonar::wall_follow_fancy_node_paramsConfig config_;

    ros::NodeHandle node_;
    ros::Publisher pub_circles_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_path_;

    ros::Subscriber sub_global_sonar_;
    ros::Subscriber sub_pos_;
    /*!
     * \brief publish circle and chosen path
     * \param circle_points points that will be published on "sonar/fancy/circles"
     * \param path_points points that will be published on "sonar/fancy/path"
     */
    void publishDebugInfo(const std::list<Vector3d> &circle_points, const std::vector<Vector3d> &path_points);

    /*!
     * \brief setup the subscribers
     *  sub_laser_ subscribes to "sonar/global_sonar/polygon"
     *  sub_pos_ subscribes on "posemeter" (active simulation) or on "position/estimate" (real world)
     */
    void setupSubscribers();

    /*!
     * \brief calculating if point lies behind robot
     * \param p Point to check
     * \param robot_yaw_angle
     * \param robot_position
     * \return true if point lies behind robot
     */
    bool isBehindRobot(const Vector3d &p, const double &robot_yaw_angle, const Vector3d &robot_position);
    /*!
     * \brief check if point lies within an other bounding circle
     * \param distance radius of the bounding circles
     * \param global_sonar_points center points of bounding circles
     * \param p point to check
     * \return true, if pint lies within a circle
     */
    bool isInsideOtherCircle(const double &distance, const std::vector<Vector3d> &global_sonar_points, const Vector3d &p);

};

#endif // WALL_FOLLOW_FANCY_NODE_H
