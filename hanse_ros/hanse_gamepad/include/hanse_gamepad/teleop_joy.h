#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include "hanse_gamepad/GamepadNodeConfig.h"
#include "hanse_srvs/EngineCommand.h"

#define NUM_SERVICE_LOOPS 8

class TeleopHanse
{
public:
    TeleopHanse();

private:
    ros::NodeHandle node;

    ros::Timer publishTimer;

    ros::ServiceClient srvClEngineCommandDepth;
    ros::ServiceClient srvClEngineCommandOrientation;

    ros::Publisher pubCmdVel;
    ros::Subscriber subJoyInput;

    double linearValue;
    double angularValue;
    double depthValue;

    bool motorsEnabled;
    bool pidsEnabled;
    bool emergencyStop;
    bool gamepadEnabled;

    bool depthUpLast;
    bool depthDownLast;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void timerCallback(const ros::TimerEvent &e);

    void dynReconfigureCallback(hanse_gamepad::GamepadNodeConfig &config, uint32_t level);
    hanse_gamepad::GamepadNodeConfig config;

    /** \brief dynamic_reconfigure interface */
    dynamic_reconfigure::Server<hanse_gamepad::GamepadNodeConfig> dynReconfigureSrv;

    /** \brief dynamic_reconfigure call back */
    dynamic_reconfigure::Server<hanse_gamepad::GamepadNodeConfig>::CallbackType dynReconfigureCb;

};
