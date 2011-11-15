#include "ros/ros.h"
#include "hanse_msgs/ScanningSonar.h"
#include "sonardriver.h"


SonarDriver::SonarDriver(ros::NodeHandle handle) :
    nh(handle),
    publisher(handle.advertise<hanse_msgs::ScanningSonar>("scanning_sonar", 1000)),
    serialSource("/dev/ttyUSB1")
{
}

void SonarDriver::tick()
{
    SonarReturnData returnData = serialSource.getNextPacket();

    hanse_msgs::ScanningSonar msg;

    QByteArray echoData = returnData.getEchoData();

    for (QByteArray::const_iterator i = echoData.begin(); i != echoData.end(); ++i) {
        msg.echoData.push_back(*i);
    }

    msg.headPosition = returnData.getHeadPosition();
    msg.range = returnData.getRange();
    msg.startGain = returnData.switchCommand.startGain;

    publisher.publish(msg);
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
    return 0;
}
