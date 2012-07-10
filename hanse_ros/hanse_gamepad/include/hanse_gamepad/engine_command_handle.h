#ifndef ENGINE_COMMAND_HANDLE_H
#define ENGINE_COMMAND_HANDLE_H

#include <ros/ros.h>

#include "hanse_srvs/EngineCommand.h"

#define NUM_SERVICE_LOOPS 4

class EngineCommandHandle
{
public:
    EngineCommandHandle(const std::string& service_name);

    static bool init();

    static void toggleMotors();
    static bool isMotorsEnabled();

    static void resetZeroPressure();
    static bool isZeroPressureResetEnabled();

    static void toggleEmergencyStop();
    static bool isEmergencyStopChanged();
    static bool isEmergencyStopSet();

    bool sendMsg();
    static void resetMsg();

    static void printChanges();
    static void recoverState();

private:
    ros::NodeHandle nh_;

    static hanse_srvs::EngineCommand msg_;
    static bool __init;

    static bool motors_changed_;
    static bool pressure_reset_changed_;
    static bool emergency_stop_changed_;

    ros::ServiceClient service_;
};

#endif // ENGINE_COMMAND_HANDLE_H
