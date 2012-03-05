#ifndef HANSE_SONARVISUALIZE_H
#define HANSE_SONARVISUALIZE_H

#include "ros/ros.h"
#include "hanse_msgs/ScanningSonar.h"

class SonarVisualize
{
public:
    SonarVisualize(ros::NodeHandle handle);

    void tick();

private:

    void callback(const hanse_msgs::ScanningSonar &msg);

    ros::NodeHandle nh;
    ros::Subscriber subscriber;
};

#endif
