#include "hanse_gamepad/engine_command_handle.h"

EngineCommandHandle::EngineCommandHandle(const std::string &service_name)
{
    service_ = nh_.serviceClient<hanse_srvs::EngineCommand>(service_name);
}

bool EngineCommandHandle::init()
{
    msg_.request.enableDepthPid = true;
    msg_.request.enableOrientationPid = true;
    msg_.request.enableMotors = true;
    msg_.request.resetZeroPressure = false;
    msg_.request.setEmergencyStop = false;

    return true;
}

void EngineCommandHandle::resetMsg()
{
    motors_changed_ = false;
    pressure_reset_changed_ = false;
    emergency_stop_changed_ = false;

    msg_.request.resetZeroPressure = false;
}

void EngineCommandHandle::toggleMotors()
{
    motors_changed_ = true;
    msg_.request.enableMotors = !msg_.request.enableMotors;
}

bool EngineCommandHandle::isMotorsEnabled()
{
    return msg_.request.enableMotors;
}

void EngineCommandHandle::resetZeroPressure()
{
    pressure_reset_changed_ = true;
    msg_.request.resetZeroPressure = true;
}

bool EngineCommandHandle::isZeroPressureResetEnabled()
{
    return msg_.request.resetZeroPressure;
}

void EngineCommandHandle::toggleEmergencyStop()
{
    emergency_stop_changed_ = true;
    msg_.request.setEmergencyStop = !msg_.request.setEmergencyStop;
}

bool EngineCommandHandle::isEmergencyStopChanged()
{
    return emergency_stop_changed_;
}

bool EngineCommandHandle::isEmergencyStopSet()
{
    return msg_.request.setEmergencyStop;
}

bool EngineCommandHandle::sendMsg()
{
    if (!(motors_changed_ || pressure_reset_changed_ || emergency_stop_changed_))
    {
        return true;
    }

    uint8_t counter = 0;
    ros::Rate loop_rate(4);

    while(ros::ok() && counter < NUM_SERVICE_LOOPS)
    {
        if (service_.call(msg_))
        {
            return true;
        }

        counter++;
        loop_rate.sleep();
    }

    return false;
}

void EngineCommandHandle::printChanges()
{
    if (motors_changed_)
    {
        if(msg_.request.enableMotors)
        {
            ROS_INFO("Motors enabled.");
        }
        else
        {
            ROS_INFO("Motors disabled.");
        }
    }

    if (pressure_reset_changed_)
    {
        ROS_INFO("Resetted pressure zero value.");
    }

    if (emergency_stop_changed_)
    {
        if (msg_.request.setEmergencyStop)
        {
            ROS_INFO("Emergency stop!");
        }
        else
        {
            ROS_INFO("Emergency stop disabled");
        }
    }
}

void EngineCommandHandle::recoverState()
{
    if (motors_changed_)
    {
        msg_.request.enableMotors = !msg_.request.enableMotors;
    }

    if (pressure_reset_changed_)
    {
        msg_.request.resetZeroPressure = false;
    }

    if (emergency_stop_changed_)
    {
        msg_.request.setEmergencyStop = !msg_.request.setEmergencyStop;
    }

    resetMsg();
}

bool EngineCommandHandle::motors_changed_ = false;
bool EngineCommandHandle::pressure_reset_changed_ = false;
bool EngineCommandHandle::emergency_stop_changed_ = false;

hanse_srvs::EngineCommand EngineCommandHandle::msg_;

bool EngineCommandHandle::__init = EngineCommandHandle::init();
