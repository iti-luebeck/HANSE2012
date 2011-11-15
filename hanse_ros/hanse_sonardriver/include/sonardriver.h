#ifndef HANSE_SONARDRIVER_H
#define HANSE_SONARDRIVER_H

#include "ros/ros.h"
#include "sonardatasourceserial.h"


class SonarDriver
{
public:
    SonarDriver(ros::NodeHandle handle);

    void tick();

private:
    ros::NodeHandle nh;
    ros::Publisher publisher;
    SonarDataSourceSerial serialSource;
};

#endif
