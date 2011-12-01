#include "scanningsonardriver.h"

ScanningSonarDriver::ScanningSonarDriver(ros::NodeHandle handle) :
    SonarDriver(handle, "scanningsonar")
{
}

void ScanningSonarDriver::updateSwitchCommand(ScanningSonarSwitchCommand &cmd,
                                              const hanse_sonardriver::ScanningSonarConfig &config)
{
    cmd.trainAngle = config.train_angle;
    cmd.sectorWidth = config.sector_width;
    cmd.stepSize = config.step_size;
    cmd.frequency = config.frequency;
}

void ScanningSonarDriver::completeMessage(hanse_msgs::ScanningSonar &msg,
                                          const SonarReturnData &returnData)
{
    msg.headPosition = returnData.getHeadPosition();
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "scanningsonardriver");
    ros::NodeHandle n;

    ScanningSonarDriver sonarDriver(n);

    while (ros::ok()) {
        sonarDriver.tick();
        ros::spinOnce();
    }
    return 0;
}
