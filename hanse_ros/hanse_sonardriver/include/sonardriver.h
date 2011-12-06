#ifndef HANSE_SONARDRIVER_H
#define HANSE_SONARDRIVER_H

#include "ros/ros.h"

#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/publisher.h"
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

    double min_freq;
    double max_freq;
    ros::Publisher publisher;
    diagnostic_updater::DiagnosedPublisher<Message> diagnosedPublisher;
    SonarDataSourceSerial serialSource;
};

#include "sonardriver.impl.h"

#endif
