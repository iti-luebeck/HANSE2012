#ifndef HANSE_SONARDRIVER_H
#define HANSE_SONARDRIVER_H

#include "ros/ros.h"

#include "diagnostic_updater/diagnostic_updater.h"
#include "dynamic_reconfigure/server.h"

#include "sonardatasourceserial.h"

template <class Config, class Message, class SwitchCommand>
class SonarDriver
{
public:
    SonarDriver(ros::NodeHandle handle, const std::string &topicName);

    void tick();

protected:
    virtual void updateSwitchCommand(SwitchCommand &cmd, const Config &config) = 0;
    virtual void completeMessage(Message &msg, const SonarReturnData &returnData) = 0;

private:
    diagnostic_updater::Updater diagnosticUpdater;

    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &status);
    void reconfigure(Config &newConfig, uint32_t level);

    ros::NodeHandle nh;

    dynamic_reconfigure::Server<Config> reconfigServer;
    Config config;
    SwitchCommand switchCommand;

    ros::Publisher publisher;
    SonarDataSourceSerial serialSource;
};

#include "sonardriver.impl.h"

#endif
