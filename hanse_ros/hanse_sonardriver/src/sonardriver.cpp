#include "ros/ros.h"
#include "hanse_msgs/ScanningSonar.h"
#include "sonardriver.h"


SonarDriver::SonarDriver(ros::NodeHandle handle) :
    nh(handle),
    publisher(handle.advertise<hanse_msgs::ScanningSonar>("scanning_sonar", 1000)),
    serialSource("/dev/ttyUSB1")
{
    diagnosticUpdater.setHardwareID("none");
    diagnosticUpdater.add("Method updater", this, &SonarDriver::produce_diagnostics);
    reconfigServer.setCallback(boost::bind(&SonarDriver::reconfigure, this, _1, _2));
}

void SonarDriver::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    //struct stat status;
    //stat(argv[1], &status);
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "device not found");
}

void SonarDriver::tick()
{    
    SonarReturnData returnData = serialSource.getNextPacket(config);

    hanse_msgs::ScanningSonar msg;

    QByteArray echoData = returnData.getEchoData();

    for (QByteArray::const_iterator i = echoData.begin(); i != echoData.end(); ++i) {
        msg.echoData.push_back(*i);
    }

    msg.headPosition = returnData.getHeadPosition();
    msg.range = returnData.getRange();
    msg.startGain = returnData.switchCommand.startGain;

    publisher.publish(msg);

    diagnosticUpdater.update();
}

void SonarDriver::reconfigure(hanse_sonardriver::ScanningSonarConfig &newConfig, uint32_t level)
{
    config = newConfig;
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
