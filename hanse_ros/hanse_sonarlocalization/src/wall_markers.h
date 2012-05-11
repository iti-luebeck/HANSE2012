#ifndef WALL_MARKERS_H
#define WALL_MARKERS_H

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <hanse_msgs/WallDetection.h>
#include <visualization_msgs/Marker.h>

class WallMarkers {
public:
    WallMarkers(ros::NodeHandle handle);

private:


    ros::NodeHandle nh;

    ros::Publisher markerPublisher;
    ros::Subscriber positionSubscriber;
    ros::Subscriber wallSubscriber;

    void positionCallback(const geometry_msgs::PoseStamped &position);
    void wallCallback(const hanse_msgs::WallDetection &wall);

    bool havePosition;
    geometry_msgs::Pose position;
    int frameID;

    std::deque<visualization_msgs::Marker> lineMarkers;
    std::deque<visualization_msgs::Marker> wallMarkers;
};

#endif
