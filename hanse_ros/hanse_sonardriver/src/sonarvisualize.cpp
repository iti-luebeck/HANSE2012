#include "sonarvisualize.h"

SonarVisualize::SonarVisualize(ros::NodeHandle handle) :
    nh(handle),
    subscriber(handle.subscribe("/hanse/sonar/scan", 1, &SonarVisualize::callback, this))
{
}

void SonarVisualize::tick()
{
}

void SonarVisualize::callback(const hanse_msgs::ScanningSonar &msg)
{
    const char gradient[10] = " .:;+=xX$";

    std::cout << std::setw(10) << msg.headPosition;
    std::cout << std::setw(10) << (int)msg.range;
    std::cout << std::setw(10) << (int)msg.startGain;

    int skip = 0;
    for (std::vector<uint8_t>::const_iterator i = msg.echoData.begin(); i != msg.echoData.end(); ++i) {
        if (skip == 0) {
            std::cout << gradient[*i / 29];
            skip = 4;
        }
        skip--;
    }
    std::cout << std::endl;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "sonarvisualize");
    ros::NodeHandle n;

    SonarVisualize sonarVisualize(n);

    while (ros::ok()) {
        sonarVisualize.tick();
        ros::spinOnce();
    }
    return 0;
}
