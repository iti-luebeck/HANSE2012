#include <sys/stat.h>

template <class C, class M, class SC>
SonarDriver<C, M, SC>::SonarDriver(ros::NodeHandle handle, const std::string &topicName) :
    isInitialized(false),
    nh(handle),
    paramHelper(),
    publisher(handle.advertise<M>(topicName, 1000)),
    diagnosedPublisher(publisher, diagnosticUpdater, diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq), diagnostic_updater::TimeStampStatusParam()),
    serialSource(QString::fromStdString(paramHelper.serialPort))
{
    min_freq = 1;
    max_freq = 1000;
    diagnosticUpdater.setHardwareID("none");
    diagnosticUpdater.add("Method updater", this, &SonarDriver::produce_diagnostics);
}

template <class C, class M, class SC>
void SonarDriver<C, M, SC>::init()
{
  isInitialized = true;
  reconfigServer.setCallback(boost::bind(&SonarDriver::reconfigure, this, _1, _2));
}

template <class C, class M, class SC>
void SonarDriver<C, M, SC>::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &status)
{
    struct stat sta;
    if(stat(paramHelper.serialPort.c_str(), &sta) == -1)
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "device not found");
    else
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "device found");
}

template <class C, class M, class SC>
void SonarDriver<C, M, SC>::tick()
{
    // init aufruf erst hier, da boost::bind mit templatemethode sonst schiefgeht
    if(!isInitialized)
      init();

    SonarReturnData returnData = serialSource.getNextPacket(switchCommand);

    if (returnData.isPacketValid()) {
        M msg;

	msg.header.stamp = ros::Time::now();

        QByteArray echoData = returnData.getEchoData();

        for (QByteArray::const_iterator i = echoData.begin(); i != echoData.end(); ++i) {
            msg.echoData.push_back(*i);
        }

        msg.range = returnData.getRange();
        msg.startGain = returnData.switchCommand.startGain;

        completeMessage(msg, returnData);

        diagnosedPublisher.publish(msg);
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
