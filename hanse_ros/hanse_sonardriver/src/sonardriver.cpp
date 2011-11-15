#include "ros/ros.h"

#include "sonardriver.h"


SonarDriver::SonarDriver(ros::NodeHandle handle) :
    nh(handle),
    serialSource("/dev/ttyUSB1")
{
}

void SonarDriver::tick()
{
    SonarReturnData returnData = serialSource.getNextPacket();
    ROS_INFO("got next packet");
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "sonardriver");
    ros::NodeHandle n;

    SonarDriver sonarDriver(n);

    while (ros::ok()) {
        sonarDriver.tick();
        ros::spinOnce();
    }
}
