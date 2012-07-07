#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>

#include "hanse_msgs/pressure.h"
#include "hanse_msgs/sollSpeed.h"
#include "hanse_srvs/Bool.h"
#include "hanse_srvs/Empty.h"
#include "hanse_srvs/EngineCommand.h"
#include "hanse_srvs/SetTarget.h"
#include "hanse_depthengine/DepthengineConfig.h"

#define NUM_SERVICE_LOOPS 8

class DepthEngine {
public:
    DepthEngine();

private:
    // Node Handle.
    ros::NodeHandle node;

    // Service for complete EngineCommand message
    ros::ServiceServer srvHandleEngineCommand;

    // Services for enabling and disabling PIDs.
    ros::ServiceServer srvEnableDepthPid;

    // Service for enabling and disabling motors completly.
    ros::ServiceServer srvEnableMotors;

    // Service for resetting the zero pressure
    ros::ServiceServer srvResetZeroPressure;

    // Service for setting emergency stop
    ros::ServiceServer srvSetEmergencyStop;

    // Service for setting the depth
    ros::ServiceServer srvSetDepth;

    // Service Clients.
    ros::ServiceClient srvClDepthPid;

    // Subscriber.
    ros::Subscriber subPressure;

    ros::Subscriber subDepthOutput;

    // Publisher.
    ros::Publisher pubDepthCurrent;
    ros::Publisher pubDepthTarget;

    ros::Publisher pubMotorFront;
    ros::Publisher pubMotorRear;

    // Daten Zwischenspeicher.
    double depthTarget;
    uint16_t pressureBias;
    uint16_t pressureCurrent;
    bool pressureInit;

    bool emergencyStop;

    bool pidsEnabled;
    bool depthPidEnabled;

    bool motorsEnabled;

    // Standard-Nachrichten.
    hanse_srvs::Bool enableMsg;
    hanse_srvs::Bool disableMsg;

    // Ausgabewerte der PID-Regler.
    int8_t depthOutput;

    double publishFrequency;

    ros::Timer publishTimer;

    // Service-Methoden
    bool handleEngineCommand(hanse_srvs::EngineCommand::Request &req,
                             hanse_srvs::EngineCommand::Response &res);
    bool enableDepthPid(hanse_srvs::Bool::Request &req,
                        hanse_srvs::Bool::Response &res);
    bool enableMotors(hanse_srvs::Bool::Request &req,
                      hanse_srvs::Bool::Response &res);
    bool resetZeroPressure(hanse_srvs::Empty::Request &req,
                           hanse_srvs::Empty::Response &res);
    bool setEmergencyStop(hanse_srvs::Bool::Request &req,
                          hanse_srvs::Bool::Response &res);
    bool setDepth(hanse_srvs::SetTarget::Request &req,
                  hanse_srvs::SetTarget::Response &res);

    // Methodendeklaration.
    void pressureCallback(const hanse_msgs::pressure::ConstPtr &pressure);

    void depthOutputCallback(const std_msgs::Float64::ConstPtr &depthOutput);

    void publishTimerCallback(const ros::TimerEvent &e);

    bool callDepthPidEnableService(const bool msg);

    void dynReconfigureCallback(hanse_depthengine::DepthengineConfig &config, uint32_t level);
    hanse_depthengine::DepthengineConfig config;

    /** \brief dynamic_reconfigure interface */
    dynamic_reconfigure::Server<hanse_depthengine::DepthengineConfig> dynReconfigureSrv;

    /** \brief dynamic_reconfigure call back */
    dynamic_reconfigure::Server<hanse_depthengine::DepthengineConfig>::CallbackType dynReconfigureCb;
};
