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
    WallFollowPublisherNode(ros::NodeHandle n);
    void configCallback(hanse_wall_sonar::wall_follow_publisher_node_paramsConfig &config, uint32_t level);
    void updateGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    hanse_wall_sonar::wall_follow_publisher_node_paramsConfig config_;
    ros::NodeHandle node_;

    ros::Publisher pub_goal_;
    ros::Subscriber sub_goal_;

    uint32_t last_publish_time_;

    std::list<geometry_msgs::Pose> last_goals_;
};

#endif // WALL_FOLLOW_PUBLISHER_NODE_H
