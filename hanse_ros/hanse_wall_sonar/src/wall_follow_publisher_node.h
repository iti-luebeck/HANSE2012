#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <list>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
#include <hanse_wall_sonar/wall_follow_publisher_node_paramsConfig.h>

#ifndef WALL_FOLLOW_PUBLISHER_NODE_H
#define WALL_FOLLOW_PUBLISHER_NODE_H

class WallFollowPublisherNode
{
public:
    /*!
     * \brief Constructs WallFollowPublisherNode.
     * Subscribes to "sonar/wall_follow/goal" to receive goals to publish on "/goal".
     * \param n Node Handle of the node
     */
    WallFollowPublisherNode(ros::NodeHandle n);
    /*!
     * \brief Config callback for dynamic reconfigure.
     * \param config object from dynamic reconfigure
     * \param level
     */
    void configCallback(hanse_wall_sonar::wall_follow_publisher_node_paramsConfig &config, uint32_t level);
    /*!
     * \brief ros callback for goal update.
     * Stores the last goals until publish time is reached. Publishes average goal of the last points.
     * \param msg goal from ros callback.
     */
    void updateGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    //! config object for dyn reconfigure
    hanse_wall_sonar::wall_follow_publisher_node_paramsConfig config_;

    ros::NodeHandle node_;

    ros::Publisher pub_goal_;
    ros::Subscriber sub_goal_;

    //! last time a goal was published.
    uint32_t last_publish_time_;

    //! list of last received goals.
    std::list<geometry_msgs::Pose> last_goals_;
};

#endif // WALL_FOLLOW_PUBLISHER_NODE_H
