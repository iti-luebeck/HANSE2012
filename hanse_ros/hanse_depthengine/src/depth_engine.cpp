#include "hanse_depthengine/depth_engine.h"

DepthEngine::DepthEngine() :
    depth_target_(0),
    pressure_bias_(0),
    pressure_current_(0),
    pressure_init_(false),
    emergency_stop_(false),
    pids_enabled_(true),
    depth_pid_enabled_(true),
    motors_enabled_(true),
    depth_output_(0)
{
    // Initialisierung der Standard-Service Nachrichten.
    enable_msg_.request.enable = true;
    disable_msg_.request.enable = false;

    // Registrierung der Publisher und Subscriber.
    pub_depth_current_ = nh_.advertise<std_msgs::Float64>("/hanse/pid/depth/input", 1);
    pub_depth_target_ = nh_.advertise<std_msgs::Float64>("/hanse/pid/depth/target",1);

    pub_motor_up_ = nh_.advertise<hanse_msgs::sollSpeed>("/hanse/motors/up", 1);

    sub_pressure_ = nh_.subscribe<hanse_msgs::pressure>("/hanse/pressure/depth", 10,
                                                       &DepthEngine::pressureCallback, this);

    sub_depth_output_ = nh_.subscribe<std_msgs::Float64>(
                "/hanse/pid/depth/output", 10, &DepthEngine::depthOutputCallback, this);

    publish_timer_ = nh_.createTimer(ros::Duration(1),
                                    &DepthEngine::publishTimerCallback, this);

    gamepad_timer_ = nh_.createTimer(ros::Duration(300),
                                      &DepthEngine::gamepadTimerCallback, this);
    sub_mux_selected_ = nh_.subscribe<std_msgs::String>("/hanse/commands/cmd_vel_mux/selected",
                                                        1, &DepthEngine::muxSelectedCallback, this);

    // Registrierung der Services.
    srv_handle_engine_command_ = nh_.advertiseService("engine/depth/handleEngineCommand",
                                                   &DepthEngine::handleEngineCommand, this);

    srv_enable_depth_pid_ = nh_.advertiseService("engine/depth/enableDepthPid",
                                              &DepthEngine::enableDepthPid, this);

    srv_enable_motors_ = nh_.advertiseService("engine/depth/enableMotors",
                                            &DepthEngine::enableMotors, this);

    srv_reset_zero_pressure_ = nh_.advertiseService("engine/depth/resetZeroPressure",
                                                 &DepthEngine::resetZeroPressure, this);

    srv_set_emergency_stop_ = nh_.advertiseService("engine/depth/setEmergencyStop",
                                                &DepthEngine::setEmergencyStop, this);

    srv_set_depth_ = nh_.advertiseService("engine/depth/setDepth", &DepthEngine::setDepth, this);
    srv_increment_depth_ = nh_.advertiseService("engine/depth/incDepth", &DepthEngine::incrementDepth, this);

    dyn_reconfigure_cb_ = boost::bind(&DepthEngine::dynReconfigureCallback, this, _1, _2);
    dyn_reconfigure_srv_.setCallback(dyn_reconfigure_cb_);

    // Registrierung der Service Clients.
    srv_client_depth_pid_  = nh_.serviceClient<hanse_srvs::Bool>("/hanse/pid/depth/enable");

    // Aktivierung des Tiefen-PID-Regler.
    if (config_.depth_pid_enabled_at_start)
    {
        if (callDepthPidEnableService(true))
        {
            ROS_INFO("Depth PID enabled.");
            depth_pid_enabled_ = true;
        }
        else
        {
            ROS_ERROR("Depth PID couldn't be enabled. Shutdown.");
            ros::shutdown();
        }
    }

    ROS_INFO("Depth engine started.");
}

void DepthEngine::dynReconfigureCallback(hanse_depthengine::DepthengineConfig &config, uint32_t level)
{
    ROS_INFO("got new parameters, level=%d", level);

    config_ = config;

    publish_timer_.setPeriod(ros::Duration(1.0/config.publish_frequency));
    gamepad_timer_.setPeriod(ros::Duration(config.gamepad_timeout));
}

void DepthEngine::velocityCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
    gamepad_timeout_ = false;

    if (gamepad_running_)
    {
        gamepad_timer_.stop();
    }

    if (twist->linear.z != 0)
    {
        depth_target_ = twist->linear.z;

        if (depth_target_ < 0)
        {
            depth_target_ = 0;
        }
    }

    if (gamepad_running_)
    {
        gamepad_timer_.start();
    }
}

// Auswertung und Zwischenspeicherung der Eingabedaten des Drucksensors.
void DepthEngine::pressureCallback(
        const hanse_msgs::pressure::ConstPtr& pressure)
{
    if (!pressure_init_)
    {
        pressure_bias_ = pressure->data;
        pressure_init_ = true;
    }

    pressure_current_ = pressure->data;
}

// Speicherung der Zieltiefe
bool DepthEngine::setDepth(
        hanse_srvs::SetTarget::Request &req,
        hanse_srvs::SetTarget::Response &res)
{
    depth_target_ = req.target;
    return true;
}

bool DepthEngine::incrementDepth(hanse_srvs::SetTarget::Request &req,
                                 hanse_srvs::SetTarget::Response &res)
{
    depth_target_ += req.target;

    if (depth_target_ < 0)
    {
        depth_target_ = 0;
    }
}

// Auswertung und Zwischenspeicherung der Ausgabedaten des Druck-PID-Reglers.
void DepthEngine::depthOutputCallback(
        const std_msgs::Float64::ConstPtr& depthOutput)
{
    this->depth_output_ = depthOutput->data;
}

void DepthEngine::muxSelectedCallback(const std_msgs::String::ConstPtr &topic)
{
    if (topic->data.find("cmd_vel_joystick") != std::string::npos)
    {
        gamepad_running_ = true;
        gamepad_timer_.start();
    }
    else
    {
        gamepad_running_ = false;
        gamepad_timer_.stop();
    }
}

void DepthEngine::gamepadTimerCallback(const ros::TimerEvent &e)
{
    gamepad_timer_.stop();
    gamepad_timeout_ = true;
    depth_target_ = 0;
    ROS_INFO("Gamepad connection lost. Come up.");
}

// Ausführung jeweils nach Ablauf des Timers. Wird verwendet, um sämtliche
// Ausgaben auf die vorgesehenen Topics zu schreiben.
void DepthEngine::publishTimerCallback(const ros::TimerEvent &e)
{
    hanse_msgs::sollSpeed motor_height_msg;
    motor_height_msg.header.stamp = ros::Time::now();

    if(emergency_stop_ || gamepad_timeout_)
    {
        motor_height_msg.data = 64;
    }
    else
    {
        if (motors_enabled_)
        {
            // Tiefensteuerung.
            std_msgs::Float64 depth_target_msg;
            depth_target_msg.data = depth_target_ + pressure_bias_;

            if (depth_target_msg.data < config_.min_depth_pressure)
            {
                depth_target_msg.data = config_.min_depth_pressure;
            }
            else if(depth_target_msg.data > config_.max_depth_pressure)
            {
                depth_target_msg.data = config_.max_depth_pressure;
            }

            pub_depth_target_.publish(depth_target_msg);

            std_msgs::Float64 depth_current_msg;
            depth_current_msg.data = pressure_current_;

            pub_depth_current_.publish(depth_current_msg);

            motor_height_msg.data = -depth_output_;
        }
        else
        {
            motor_height_msg.data = 0;
        }
    }

    pub_motor_up_.publish(motor_height_msg);
}

bool DepthEngine::handleEngineCommand(hanse_srvs::EngineCommand::Request &req,
                                        hanse_srvs::EngineCommand::Response &res)
{
    if (req.setEmergencyStop && !emergency_stop_)
    {
        emergency_stop_ = true;
        pids_enabled_ = false;

        // Deaktivierung des Tiefen-PID-Reglers.
        if(depth_pid_enabled_)
        {
            if (callDepthPidEnableService(false))
            {
                ROS_INFO("Depth PID disabled.");
            }
            else
            {
                ROS_ERROR("Depth PID couldn't be disabled.");
            }
        }

        // Tiefe auf 0 setzen
        depth_target_ = 0;
    }
    else if (!req.setEmergencyStop)
    {
        if (emergency_stop_)
        {
            emergency_stop_ = false;
            pids_enabled_ = true;

            if (req.enableDepthPid)
            {
                if (callDepthPidEnableService(true))
                {
                    ROS_INFO("Depth PID enabled.");
                }
                else
                {
                    ROS_ERROR("Depth PID couldn't be enabled.");
                }
            }

            depth_pid_enabled_ = req.enableDepthPid;
        }
        else
        {
            if (!depth_pid_enabled_ && req.enableDepthPid)
            {
                if (callDepthPidEnableService(true))
                {
                    ROS_INFO("Depth PID enabled.");
                    depth_pid_enabled_ = true;
                }
                else
                {
                    ROS_ERROR("Depth PID couldn't be enabled.");
                }
            }
            else if (depth_pid_enabled_ && !req.enableDepthPid)
            {
                if (callDepthPidEnableService(false))
                {
                    ROS_INFO("Depth PID disabled.");
                    depth_pid_enabled_ = false;
                }
                else
                {
                    ROS_ERROR("Depth PID couldn't be disabled.");
                }
            }
        }

        if (motors_enabled_ != req.enableMotors)
        {
            if (req.enableMotors)
            {
                ROS_INFO("Motors enabled.");
            }
            else
            {
                ROS_INFO("Motors disabled.");
            }

            motors_enabled_ = req.enableMotors;
        }

        if (req.resetZeroPressure)
        {
            ROS_INFO("Resetting zero pressure.");
            pressure_init_ = false;
        }
    }

    return true;
}

bool DepthEngine::enableDepthPid(hanse_srvs::Bool::Request &req,
                                   hanse_srvs::Bool::Response &res)
{
    if (pids_enabled_) {
        if (!depth_pid_enabled_ && req.enable)
        {
            if (callDepthPidEnableService(true))
            {
                ROS_INFO("Depth PID enabled.");
                depth_pid_enabled_ = true;
            }
            else
            {
                ROS_ERROR("Depth PID couldn't be enabled.");
            }
        }
        else if (depth_pid_enabled_ && !req.enable)
        {
            if (callDepthPidEnableService(false))
            {
                ROS_INFO("Depth PID disabled.");
                depth_pid_enabled_ = false;
            }
            else
            {
                ROS_ERROR("Depth PID couldn't be disabled.");
            }
        }
    }
    else
    {
        depth_pid_enabled_ = req.enable;
    }

    return true;
}

bool DepthEngine::enableMotors(hanse_srvs::Bool::Request &req,
                                 hanse_srvs::Bool::Response &res)
{
    motors_enabled_ = req.enable;
    return true;
}

bool DepthEngine::resetZeroPressure(hanse_srvs::Empty::Request &req,
                                      hanse_srvs::Empty::Response &res)
{
    pressure_init_ = false;
    return true;
}

bool DepthEngine::setEmergencyStop(hanse_srvs::Bool::Request &req,
                                     hanse_srvs::Bool::Response &res)
{
    if (req.enable && !emergency_stop_)
    {
        emergency_stop_ = true;
        pids_enabled_ = false;

        // Deaktivierung des Tiefen-PID-Reglers.
        if(depth_pid_enabled_) {
            if (callDepthPidEnableService(false))
            {
                ROS_INFO("Depth PID disabled.");
            }
            else
            {
                ROS_ERROR("Depth PID couldn't be disabled.");
            }
        }

        // Tiefe auf 0 setzen
        depth_target_ = 0;
    }
    else if (!req.enable && emergency_stop_)
    {
        emergency_stop_ = false;
        pids_enabled_ = true;

        // Aktivierung der zuvor aktivierten PID Regler
        if (depth_pid_enabled_)
        {
            if (callDepthPidEnableService(true))
            {
                ROS_INFO("Depth PID enabled.");
            }
            else
            {
                ROS_ERROR("Depth PID couldn't be enabled.");
            }
        }
    }

    return true;
}

bool DepthEngine::callDepthPidEnableService(const bool msg)
{
    int8_t counter = 0;
    ros::Rate loop_rate(4);
    bool success = false;

    while(ros::ok() && counter < NUM_SERVICE_LOOPS)
    {
        if ((msg && srv_client_depth_pid_.call(enable_msg_)) ||
                (!msg && srv_client_depth_pid_.call(disable_msg_)))
        {
            success = true;
            break;
        }

        counter++;
        loop_rate.sleep();
    }

    return success;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthengine_node");

    ros::start();

    DepthEngine engine_control;

    ros::spin();
}
