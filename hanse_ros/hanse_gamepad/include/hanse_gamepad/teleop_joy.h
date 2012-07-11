#ifndef TELEOP_JOY_H
#define TELEOP_JOY_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <topic_tools/MuxSelect.h>

#include "hanse_gamepad/GamepadNodeConfig.h"
#include "hanse_gamepad/flank_trigger.h"
#include "hanse_gamepad/engine_command_handle.h"
#include "hanse_srvs/EngineCommand.h"
#include "hanse_srvs/SetTarget.h"

#define NUM_SERVICE_LOOPS 4

class TeleopHanse
{
public:
    TeleopHanse();

private:
    ros::NodeHandle nh_;

    ros::Timer publish_timer_;
    ros::Timer delay_timer_;

    ros::ServiceClient srv_client_engine_inc_depth_;
    ros::ServiceClient srv_vel_mux_select_;

    ros::Publisher pub_cmd_vel_;
    ros::Subscriber sub_joy_input_;

    double linear_value_;
    double angular_value_;

    bool motors_enabled_;
    bool pids_enabled_;
    bool emergency_stop_;
    bool gamepad_enabled_;

    bool gamepad_delay_ended_;

    FlankTrigger trig_;
    EngineCommandHandle depth_cmd_h_;
    EngineCommandHandle orientation_cmd_h_;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void timerCallback(const ros::TimerEvent &e);
    void delayCallback(const ros::TimerEvent &e);
    bool switchGamepadUsage(const char* commandVelocityTopic);
    void handleGamepadMove(std::vector<float> axes);
    bool sendTargetDepth(const double increment);
    void sendEngineCommands();

    void dynReconfigureCallback(hanse_gamepad::GamepadNodeConfig &config_, uint32_t level);
    hanse_gamepad::GamepadNodeConfig config_;

    /** \brief dynamic_reconfigure interface */
    dynamic_reconfigure::Server<hanse_gamepad::GamepadNodeConfig> dyn_reconfigure_srv_;

    /** \brief dynamic_reconfigure call back */
    dynamic_reconfigure::Server<hanse_gamepad::GamepadNodeConfig>::CallbackType dyn_reconfigure_cb_;
};

#endif // TELEOP_JOY_H
