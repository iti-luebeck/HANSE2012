#ifndef HANSE_SONARDRIVER_H
#define HANSE_SONARDRIVER_H

#include "diagnostic_updater/diagnostic_updater.h"

#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "hanse_sonardriver/ScanningSonarConfig.h"

#include "sonardatasourceserial.h"


class SonarDriver
{
public:
    SonarDriver(ros::NodeHandle handle);

    void tick();

private:
    diagnostic_updater::Updater diagnosticUpdater;

    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void reconfigure(hanse_sonardriver::ScanningSonarConfig &newConfig, uint32_t level);

    ros::NodeHandle nh;

    dynamic_reconfigure::Server<hanse_sonardriver::ScanningSonarConfig> reconfigServer;
    hanse_sonardriver::ScanningSonarConfig config;

    ros::Publisher publisher;
    SonarDataSourceSerial serialSource;
};

#endif
