#ifndef ONLINE_WALL_DETECTION_H
#define ONLINE_WALL_DETECTION_H

#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "hanse_msgs/ScanningSonar.h"

class OnlineWallDetection
{
public:
    OnlineWallDetection(ros::NodeHandle handle);

private:
    class WallDistance
    {
    public:
	double headPosition;
	double distance;
    };

    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Subscriber subscriber;

    void callback(const hanse_msgs::ScanningSonar &msg);
    WallDistance computeWallDistance(const hanse_msgs::ScanningSonar &msg);
};

#endif
