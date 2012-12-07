#ifndef WALL_DETECTION_H
#define WALL_DETECTION_H

#include <cmath>
#include <map>
#include "ros/ros.h"
#include "hanse_msgs/ScanningSonar.h"
#include "hanse_msgs/ELaserScan.h"
#include "hanse_sonarlocalization/WallDetectionConfig.h"
#include "dynamic_reconfigure/server.h"

class WallDetection
{
public:
    WallDetection(ros::NodeHandle handle);

private:

    class WallDistance
    {
    public:
	double headPosition;
	double distance;
    };

    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Publisher ePublisher;
    ros::Subscriber subscriber;
    std::map<double, WallDistance> sonarDataMap;
    double lastHeadPosition;
    double movedSincePublish;
    double publishAngle;
    double stepSize;
    bool isInitialized;
    dynamic_reconfigure::Server<hanse_sonarlocalization::WallDetectionConfig> reconfigServer;

    void init();
    void reconfigure(hanse_sonarlocalization::WallDetectionConfig &newConfig, uint32_t level);

    void callback(const hanse_msgs::ScanningSonar &msg);
    WallDistance computeWallDistance(const hanse_msgs::ScanningSonar &msg);
    void publishLaserScan(ros::Time stamp);

};

#endif
