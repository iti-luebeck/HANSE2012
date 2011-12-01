#include <sys/stat.h>

template <class C, class M, class SC>
SonarDriver<C, M, SC>::SonarDriver(ros::NodeHandle handle, const std::string &topicName) :
    nh(handle),
    publisher(handle.advertise<M>(topicName, 1000)),
    serialSource("/dev/ttyUSB1")
{
    diagnosticUpdater.setHardwareID("none");
    diagnosticUpdater.add("Method updater", this, &SonarDriver::produce_diagnostics);
    reconfigServer.setCallback(boost::bind(&SonarDriver::reconfigure, this, _1, _2));
}

template <class C, class M, class SC>
void SonarDriver<C, M, SC>::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &status)
{
    struct stat sta;
    if(stat("/dev/ttyUSB0", &sta) == -1)
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "device not found");
    else
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "device found");
}

template <class C, class M, class SC>
void SonarDriver<C, M, SC>::tick()
{
    SonarReturnData returnData = serialSource.getNextPacket(switchCommand);

    if (returnData.isPacketValid()) {
        M msg;

        QByteArray echoData = returnData.getEchoData();

        for (QByteArray::const_iterator i = echoData.begin(); i != echoData.end(); ++i) {
            msg.echoData.push_back(*i);
        }

        msg.range = returnData.getRange();
        msg.startGain = returnData.switchCommand.startGain;

        completeMessage(msg, returnData);

        publisher.publish(msg);
    }
    diagnosticUpdater.update();
}

template <class C, class M, class SC>
void SonarDriver<C, M, SC>::reconfigure(C &newConfig, uint32_t level)
{
    config = newConfig;

    switchCommand.range = config.range;
    switchCommand.startGain = config.start_gain;
    switchCommand.pulseLength = config.pulse_length;
    switchCommand.dataPoints = config.data_points;
    switchCommand.switchDelay = config.switch_delay;
    updateSwitchCommand(switchCommand, config);
}
