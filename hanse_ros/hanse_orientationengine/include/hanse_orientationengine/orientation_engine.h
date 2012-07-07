#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>

#include <Eigen/Geometry>

#include "hanse_orientationengine/OrientationengineConfig.h"
#include "hanse_msgs/pressure.h"
#include "hanse_msgs/sollSpeed.h"
#include "hanse_srvs/Bool.h"
#include "hanse_srvs/Empty.h"
#include "hanse_srvs/EngineCommand.h"

#define NUM_SERVICE_LOOPS 8

class OrientationEngine {
public:
    OrientationEngine();

private:
    // Node Handle.
    ros::NodeHandle node;

    // Service for complete EngineCommand message
    ros::ServiceServer srvHandleEngineCommand;

    // Services for enabling and disabling PIDs.
    ros::ServiceServer srvEnableOrientationPid;

    // Service for enabling and disabling motors completly.
    ros::ServiceServer srvEnableMotors;

    // Service for setting emergency stop
    ros::ServiceServer srvSetEmergencyStop;

    // Service Clients.
    ros::ServiceClient srvClOrientationPid;

    // Subscriber.
    ros::Subscriber subVelocity;
    ros::Subscriber subXsens;

    ros::Subscriber subOrientationOutput;

    // Publisher.
    ros::Publisher pubOrientationCurrent;
    //ros::Publisher pubOrientationTarget;

    ros::Publisher pubMotorLeft;
    ros::Publisher pubMotorRight;

    // Daten Zwischenspeicher.
    double linearSpeed;
    double angularSpeed;

    double orientationCurrent;
    double orientationTarget;
    bool orientationInit;

    bool emergencyStop;

    bool pidsEnabled;
    bool orientationPidEnabled;

    bool motorsEnabled;

    // Standard-Nachrichten.
    hanse_srvs::Bool enableMsg;
    hanse_srvs::Bool disableMsg;

    // Ausgabewerte der PID-Regler.
    int8_t orientationOutput;

    double publishFrequency;

    ros::Timer publishTimer;

    // Service-Methoden
    bool handleEngineCommand(hanse_srvs::EngineCommand::Request &req,
                             hanse_srvs::EngineCommand::Response &res);
    bool enableOrientationPid(hanse_srvs::Bool::Request &req,
                                hanse_srvs::Bool::Response &res);
    bool enableMotors(hanse_srvs::Bool::Request &req,
                      hanse_srvs::Bool::Response &res);
    bool setEmergencyStop(hanse_srvs::Bool::Request &req,
                          hanse_srvs::Bool::Response &res);

    // Methodendeklaration.
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& twist);
    void xsensCallback(const sensor_msgs::Imu::ConstPtr& xsensData);

    void orientationOutputCallback(const std_msgs::Float64::ConstPtr& orientationOutput);

    void publishTimerCallback(const ros::TimerEvent &e);

    bool callOrientationPidEnableService(const bool msg);

    void dynReconfigureCallback(hanse_orientationengine::OrientationengineConfig &config, uint32_t level);
    hanse_orientationengine::OrientationengineConfig config;

    /** \brief dynamic_reconfigure interface */
    dynamic_reconfigure::Server<hanse_orientationengine::OrientationengineConfig> dynReconfigureSrv;

    /** \brief dynamic_reconfigure call back */
    dynamic_reconfigure::Server<hanse_orientationengine::OrientationengineConfig>::CallbackType dynReconfigureCb;
};
