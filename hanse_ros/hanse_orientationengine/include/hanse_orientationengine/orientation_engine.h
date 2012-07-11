#ifndef ORIENTATION_ENGINE_H
#define ORIENTATION_ENGINE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
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

#define NUM_SERVICE_LOOPS 4

class OrientationEngine {
public:
    OrientationEngine();

private:
    // Node Handle.
    ros::NodeHandle nh_;

    // Service for complete EngineCommand message
    ros::ServiceServer srv_handle_engine_command_;

    // Services for enabling and disabling PIDs.
    ros::ServiceServer srv_enable_orientation_pid_;

    // Service for enabling and disabling motors completly.
    ros::ServiceServer srv_enable_motors_;

    // Service for setting emergency stop
    ros::ServiceServer srv_set_emergency_stop_;

    // Service Clients.
    ros::ServiceClient srv_client_orientation_pid_;

    // Subscriber.
    ros::Subscriber sub_velocity_;
    ros::Subscriber sub_xsens_;
    ros::Subscriber sub_mux_selected_;

    ros::Subscriber sub_orientation_output_;

    // Publisher.
    ros::Publisher pub_orientation_current_;

    ros::Publisher pub_motor_left_;
    ros::Publisher pub_motor_right_;

    // Daten Zwischenspeicher.
    double linear_speed_;
    double angular_speed_;

    double orientation_current_;
    double orientation_target_;
    bool orientation_init_;

    bool emergency_stop_;

    bool pids_enabled_;
    bool orientation_pid_enabled_;

    bool motors_enabled_;

    bool gamepad_running_;

    bool turn_timer_started_;
    bool turn_timer_ended_;

    // Standard-Nachrichten.
    hanse_srvs::Bool enable_msg_;
    hanse_srvs::Bool disable_msg_;

    // Ausgabewerte der PID-Regler.
    int8_t orientation_output_;

    ros::Timer publish_timer_;
    ros::Timer gamepad_timer_;
    ros::Timer turn_timer_;

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

    void orientationOutputCallback(const std_msgs::Float64::ConstPtr& orientation_output_);

    void publishTimerCallback(const ros::TimerEvent &e);

    void muxSelectedCallback(const std_msgs::String::ConstPtr& topic);
    void gamepadTimerCallback(const ros::TimerEvent &e);

    void turnTimerCallback(const ros::TimerEvent &e);

    bool callOrientationPidEnableService(const bool msg);

    void dynReconfigureCallback(hanse_orientationengine::OrientationengineConfig &config_, uint32_t level);
    hanse_orientationengine::OrientationengineConfig config_;

    /** \brief dynamic_reconfigure interface */
    dynamic_reconfigure::Server<hanse_orientationengine::OrientationengineConfig> dyn_reconfigure_srv_;

    /** \brief dynamic_reconfigure call back */
    dynamic_reconfigure::Server<hanse_orientationengine::OrientationengineConfig>::CallbackType dyn_reconfigure_cb_;
};

#endif // ORIENTATION_ENGINE_H
