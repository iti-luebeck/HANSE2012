#ifndef DEPTH_ENGINE_H
#define DEPTH_ENGINE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>

#include "hanse_msgs/pressure.h"
#include "hanse_msgs/sollSpeed.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "hanse_srvs/Bool.h"
#include "hanse_srvs/Empty.h"
#include "hanse_srvs/EngineCommand.h"
#include "hanse_srvs/SetTarget.h"
#include "hanse_depthengine/DepthengineConfig.h"

#define NUM_SERVICE_LOOPS 4

class DepthEngine {
public:
    DepthEngine();

private:
    // Node Handle.
    ros::NodeHandle nh_;

    // Service for complete EngineCommand message
    ros::ServiceServer srv_handle_engine_command_;

    // Services for enabling and disabling PIDs.
    ros::ServiceServer srv_enable_depth_pid_;

    // Service for enabling and disabling motors completly.
    ros::ServiceServer srv_enable_motors_;

    // Service for resetting the zero pressure
    ros::ServiceServer srv_reset_zero_pressure_;

    // Service for setting emergency stop
    ros::ServiceServer srv_set_emergency_stop_;

    // Service for setting the depth
    ros::ServiceServer srv_set_depth_;

    // Service for increment and decrement depth
    ros::ServiceServer srv_increment_depth_;

    // Service Clients.
    ros::ServiceClient srv_client_depth_pid_;

    // Subscriber.
    ros::Subscriber sub_velocity_;
    ros::Subscriber sub_pressure_;
    ros::Subscriber sub_depth_output_;
    ros::Subscriber sub_mux_selected_;

    // Publisher.
    ros::Publisher pub_depth_current_;
    ros::Publisher pub_depth_target_;

    ros::Publisher pub_motor_up_;

    // Daten Zwischenspeicher.
    double depth_target_;
    uint16_t pressure_bias_;
    uint16_t pressure_current_;
    bool pressure_init_;

    bool emergency_stop_;

    bool pids_enabled_;
    bool depth_pid_enabled_;

    bool motors_enabled_;

    bool gamepad_running_;
    bool gamepad_timeout_;

    // Standard-Nachrichten.
    hanse_srvs::Bool enable_msg_;
    hanse_srvs::Bool disable_msg_;

    // Ausgabewerte der PID-Regler.
    int8_t depth_output_;

    ros::Timer publish_timer_;
    ros::Timer gamepad_timer_;

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

    bool incrementDepth(hanse_srvs::SetTarget::Request &req,
                        hanse_srvs::SetTarget::Response &res);

    // Methodendeklaration.
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& twist);
    void pressureCallback(const hanse_msgs::pressure::ConstPtr &pressure);

    void depthOutputCallback(const std_msgs::Float64::ConstPtr &depth_output_);

    void muxSelectedCallback(const std_msgs::String::ConstPtr& topic);
    void gamepadTimerCallback(const ros::TimerEvent &e);

    void publishTimerCallback(const ros::TimerEvent &e);

    bool callDepthPidEnableService(const bool msg);

    void dynReconfigureCallback(hanse_depthengine::DepthengineConfig &config_, uint32_t level);
    hanse_depthengine::DepthengineConfig config_;

    /** \brief dynamic_reconfigure interface */
    dynamic_reconfigure::Server<hanse_depthengine::DepthengineConfig> dyn_reconfigure_srv_;

    /** \brief dynamic_reconfigure call back */
    dynamic_reconfigure::Server<hanse_depthengine::DepthengineConfig>::CallbackType dyn_reconfigure_cb_;
};

#endif // DEPTH_ENGINE_H
