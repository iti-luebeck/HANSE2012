#include "echosounderdriver.h"

EchoSounderDriver::EchoSounderDriver(ros::NodeHandle handle) :
    SonarDriver(handle, "sonar/echo")
{
}

void EchoSounderDriver::updateSwitchCommand(EchoSounderSwitchCommand &cmd,
                                              const hanse_sonardriver::EchoSounderConfig &config)
{
    cmd.profileMinimumRange = config.profile_minimum_range;
    cmd.profile = config.profile;
}

void EchoSounderDriver::completeMessage(hanse_msgs::EchoSounder &msg,
                                          const SonarReturnData &returnData)
{
    msg.profileRange = returnData.getProfileRange();
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "sonar_driver_echo");
    ros::NodeHandle n;

    EchoSounderDriver sonarDriver(n);

    while (ros::ok()) {
        sonarDriver.tick();
        ros::spinOnce();
    }
    return 0;
}
