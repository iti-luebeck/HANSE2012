#include <cmath>

#include "hanse_orientationengine/orientation_engine.h"

OrientationEngine::OrientationEngine() :
    linear_speed_(0),
    angular_speed_(0),
    orientation_current_(0),
    orientation_target_(0),
    orientation_init_(false),
    emergency_stop_(false),
    pids_enabled_(true),
    orientation_pid_enabled_(false),
    motors_enabled_(true),
    gamepad_running_(false),
    turn_timer_started_(false),
    turn_timer_ended_(false),
    orientation_output_(0)
{
    // Initialisierung der Standard-Service Nachrichten.
    enable_msg_.request.enable = true;
    disable_msg_.request.enable = false;

    // Registrierung der Publisher und Subscriber.
    pub_orientation_current_ = nh_.advertise<std_msgs::Float64>(
                "/hanse/pid/orientation/input", 1);

    pub_motor_left_ = nh_.advertise<hanse_msgs::sollSpeed>("/hanse/motors/left", 1);
    pub_motor_right_ = nh_.advertise<hanse_msgs::sollSpeed>("/hanse/motors/right", 1);

    sub_velocity_ = nh_.subscribe<geometry_msgs::Twist>("/hanse/commands/cmd_vel", 10,
                                                       &OrientationEngine::velocityCallback, this);
    sub_xsens_ = nh_.subscribe<sensor_msgs::Imu>(
                "/hanse/imu", 10, &OrientationEngine::xsensCallback, this);

    sub_orientation_output_ = nh_.subscribe<std_msgs::Float64>(
                "/hanse/pid/orientation/output", 10, &OrientationEngine::orientationOutputCallback, this);

    publish_timer_ = nh_.createTimer(ros::Duration(1),
                                    &OrientationEngine::publishTimerCallback, this, false, false);
    gamepad_timer_ = nh_.createTimer(ros::Duration(300),
                                      &OrientationEngine::gamepadTimerCallback, this);
    turn_timer_ = nh_.createTimer(ros::Duration(0.2),
                                  &OrientationEngine::turnTimerCallback, this, true, false);

    sub_mux_selected_ = nh_.subscribe<std_msgs::String>("/hanse/commands/cmd_vel_mux/selected",
                                                         1, &OrientationEngine::muxSelectedCallback, this);

    // Registrierung der Services.
    srv_handle_engine_command_ = nh_.advertiseService("engine/orientation/handleEngineCommand",
                                                   &OrientationEngine::handleEngineCommand, this);

    srv_enable_orientation_pid_ = nh_.advertiseService("engine/orientation/enableOrientationPid",
                                                    &OrientationEngine::enableOrientationPid, this);

    srv_enable_motors_ = nh_.advertiseService("engine/orientation/enableMotors",
                                            &OrientationEngine::enableMotors, this);

    srv_set_emergency_stop_ = nh_.advertiseService("engine/orientation/setEmergencyStop",
                                                &OrientationEngine::setEmergencyStop, this);

    dyn_reconfigure_cb_ = boost::bind(&OrientationEngine::dynReconfigureCallback, this, _1, _2);
    dyn_reconfigure_srv_.setCallback(dyn_reconfigure_cb_);

    // Registrierung der Service Clients.
    srv_client_orientation_pid_  = nh_.serviceClient<hanse_srvs::Bool>("/hanse/pid/orientation/enable");

    if (config_.orientation_pid_enabled_at_start)
    {
        if (callOrientationPidEnableService(true))
        {
            ROS_INFO("Orientation PID enabled.");
            orientation_pid_enabled_ = true;
        }
        else
        {
            ROS_ERROR("Orientation PID couldn't be enabled. Shutdown.");
            ros::shutdown();
        }
    }

    ROS_INFO("Orientation engine started.");
}

void OrientationEngine::dynReconfigureCallback(hanse_orientationengine::OrientationengineConfig &config, uint32_t level)
{
    ROS_INFO("got new parameters, level=%d", level);

    config_ = config;

    publish_timer_.setPeriod(ros::Duration(1.0/config.publish_frequency));
    gamepad_timer_.setPeriod(ros::Duration(config.gamepad_timeout));
    turn_timer_.setPeriod(ros::Duration(config.angular_pid_delay));
}

// Auswertung und Zwischenspeicherung der cmd_vel Nachricht.
void OrientationEngine::velocityCallback(
        const geometry_msgs::Twist::ConstPtr& twist)
{
    if (gamepad_running_) {
        gamepad_timer_.stop();
    }

    linear_speed_ = twist->linear.x;
    angular_speed_ = twist->angular.z;

    if (angular_speed_ == 0 && !orientation_pid_enabled_)
    {
        if (!turn_timer_started_ && !turn_timer_ended_)
        {
            turn_timer_started_ = true;
            turn_timer_.start();
        }

        if (turn_timer_ended_)
        {
            callOrientationPidEnableService(true);
            orientation_pid_enabled_ = true;

            turn_timer_ended_ = false;
        }
    }
    else if (angular_speed_ != 0)
    {
        turn_timer_.stop();
        turn_timer_started_ = false;
        turn_timer_ended_ = false;

        if (orientation_pid_enabled_)
        {
            callOrientationPidEnableService(false);
            orientation_pid_enabled_ = false;
        }
    }

    if (gamepad_running_) {
        gamepad_timer_.start();
    }
}

// Auswertung und Zwischenspeicherung der XSens Eingabedaten.
void OrientationEngine::xsensCallback(
        const sensor_msgs::Imu::ConstPtr& imu)
{

    Eigen::Quaternionf flipped(imu->orientation.w,
                               imu->orientation.x,
                               imu->orientation.y,
                               imu->orientation.z);

    // This undos the rotation of the xsens (180 deg around the y
    // axis) this is why: we first (right side) put the xsens into hanse the
    // wrong way around and _then_ move hanse around (left side) so
    // the inverse of the xsens orientation has to be at the right
    // side too for them to cancle
    Eigen::Quaternionf orientation = flipped * Eigen::AngleAxis<float>(M_PI, Eigen::Vector3f(0, 1, 0));
    Eigen::Vector3f direction = orientation * Eigen::Vector3f(1, 0, 0);

    orientation_current_ = atan2(direction.y(), direction.x());

    if (angular_speed_ != 0 || !orientation_init_)
    {
        orientation_init_ = true;
        orientation_target_ = orientation_current_;
    }
}

// Auswertung und Zwischenspeicherung der Ausgabedaten des Orientierungs-PID-Reglers.
void OrientationEngine::orientationOutputCallback(
        const std_msgs::Float64::ConstPtr& orientation_output)
{
    orientation_output_ = orientation_output->data;
}

void OrientationEngine::muxSelectedCallback(const std_msgs::String::ConstPtr &topic)
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

void OrientationEngine::gamepadTimerCallback(const ros::TimerEvent &e)
{
    gamepad_timer_.stop();
    linear_speed_ = 0;
    angular_speed_ = 0;
    ROS_INFO("Gamepad connection lost. Stop movement.");
}

void OrientationEngine::turnTimerCallback(const ros::TimerEvent &e)
{
    turn_timer_started_ = false;
    turn_timer_ended_ = true;
}

// Ausführung jeweils nach Ablauf des Timers. Wird verwendet, um sämtliche
// Ausgaben auf die vorgesehenen Topics zu schreiben.
void OrientationEngine::publishTimerCallback(const ros::TimerEvent &e)
{
    hanse_msgs::sollSpeed motor_left_msg;
    hanse_msgs::sollSpeed motor_right_msg;
    motor_left_msg.data = 0;
    motor_right_msg.data = 0;

    if(!emergency_stop_ && motors_enabled_)
    {
         // Orientierungssteuerung.
         std_msgs::Float64 orientation_msg;
         double rotation;

         rotation = - (orientation_target_ - orientation_current_);

         if(rotation < -M_PI)
         {
             rotation = rotation + 2 * M_PI;
         }
         else if (rotation >= M_PI)
         {
             rotation = rotation - 2 * M_PI;
         }

         orientation_msg.data = rotation;
         pub_orientation_current_.publish(orientation_msg);

         // Motorsteuerung
         int16_t motor_left = 0;
         int16_t motor_right = 0;

         // Berechnung der Ansteuerungsstärke der seitlichen Motoren.
         if (angular_speed_ == 0)
         {
             motor_left = linear_speed_ * 127 - orientation_output_;
             motor_right = linear_speed_ * 127 + orientation_output_;
         }
         else
         {
             motor_left = linear_speed_ * 127 - angular_speed_ * 127;
             motor_right = linear_speed_ * 127 + angular_speed_ * 127;
         }

         // Beschränkung der Ansteuerungsstärke auf -127 bis 127.
         motor_left = std::min(motor_left, (int16_t) 127);
         motor_left = std::max(motor_left, (int16_t) -127);

         motor_right = std::min(motor_right, (int16_t) 127);
         motor_right = std::max(motor_right, (int16_t) -127);

         motor_left_msg.data = motor_left;
         motor_right_msg.data = motor_right;
    }

    pub_motor_left_.publish(motor_left_msg);
    pub_motor_right_.publish(motor_right_msg);
}

bool OrientationEngine::handleEngineCommand(hanse_srvs::EngineCommand::Request &req,
                                        hanse_srvs::EngineCommand::Response &res)
{
    if (req.setEmergencyStop && !emergency_stop_)
    {
        emergency_stop_ = true;
        pids_enabled_ = false;

        // Deaktivierung aller PID-Regler.
        if(orientation_pid_enabled_)
        {
            if (callOrientationPidEnableService(false))
            {
                ROS_INFO("Orientation PID disabled.");
            }
            else
            {
                ROS_ERROR("Orientation PID couldn't be disabled.");
            }
        }

        // Bewegung auf 0 setzen
        linear_speed_ = 0;
        angular_speed_ = 0;
    }
    else if (!req.setEmergencyStop)
    {
        if (emergency_stop_)
        {
            emergency_stop_ = false;
            pids_enabled_ = true;

            if (req.enableOrientationPid)
            {
                if (callOrientationPidEnableService(true))
                {
                    ROS_INFO("Orientation PID enabled.");
                    orientation_pid_enabled_ = true;
                }
                else
                {
                    ROS_ERROR("Orientation PID couldn't be enabled.");
                    orientation_pid_enabled_ = false;
                }
            }
        }
        else
        {
            if (!orientation_pid_enabled_ && req.enableOrientationPid)
            {
                if (callOrientationPidEnableService(true))
                {
                    ROS_INFO("Orientation PID enabled.");
                    orientation_pid_enabled_ = true;
                }
                else
                {
                    ROS_ERROR("Orientation PID couldn't be enabled.");
                }
            }
            else if (orientation_pid_enabled_ && !req.enableOrientationPid)
            {
                if (callOrientationPidEnableService(false))
                {
                    ROS_INFO("Orientation PID disabled.");
                    orientation_pid_enabled_ = false;
                }
                else
                {
                    ROS_ERROR("Orientation PID couldn't be disabled.");
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
    }

    return true;
}

bool OrientationEngine::enableOrientationPid(hanse_srvs::Bool::Request &req,
                                         hanse_srvs::Bool::Response &res)
{
    if (pids_enabled_)
    {
        if (!orientation_pid_enabled_ && req.enable)
        {
            if (callOrientationPidEnableService(true))
            {
                ROS_INFO("Orientation PID enabled.");
                orientation_pid_enabled_ = true;
            }
            else
            {
                ROS_ERROR("Orientation PID couldn't be enabled.");
            }
        }
        else if (orientation_pid_enabled_ && !req.enable)
        {
            if (callOrientationPidEnableService(false))
            {
                ROS_INFO("Orientation PID disabled.");
                orientation_pid_enabled_ = false;
            }
            else
            {
                ROS_ERROR("Orientation PID couldn't be disabled.");
            }
        }
    }
    else
    {
        orientation_pid_enabled_ = req.enable;
    }

    return true;
}

bool OrientationEngine::enableMotors(hanse_srvs::Bool::Request &req,
                                 hanse_srvs::Bool::Response &res)
{
    motors_enabled_ = req.enable;

    return true;
}

bool OrientationEngine::setEmergencyStop(hanse_srvs::Bool::Request &req,
                                     hanse_srvs::Bool::Response &res)
{
    if (req.enable && !emergency_stop_)
    {
        emergency_stop_ = true;
        pids_enabled_ = false;

        // Deaktivierung aller PID-Regler.
        if(orientation_pid_enabled_)
        {
            if (callOrientationPidEnableService(false))
            {
                ROS_INFO("Orientation PID disabled.");
            }
            else
            {
                ROS_ERROR("Orientation PID couldn't be disabled.");
            }
        }

        // Bewegung auf 0 setzen
        linear_speed_ = 0;
        angular_speed_ = 0;
    }
    else if (!req.enable && emergency_stop_)
    {
        emergency_stop_ = false;
        pids_enabled_ = true;

        if(orientation_pid_enabled_)
        {
            if (callOrientationPidEnableService(true))
            {
                ROS_INFO("Orientation PID enabled.");
            }
            else
            {
                ROS_ERROR("Orientation PID couldn't be enabled.");
            }
        }
    }

    return true;
}

bool OrientationEngine::callOrientationPidEnableService(const bool msg)
{
    int8_t counter = 0;
    ros::Rate loop_rate(4);
    bool success = false;

    while(ros::ok() && counter < NUM_SERVICE_LOOPS)
    {
        if ((msg && srv_client_orientation_pid_.call(enable_msg_)) ||
                (!msg && srv_client_orientation_pid_.call(disable_msg_)))
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
    ros::init(argc, argv, "engine_control");

    ros::start();

    OrientationEngine engine_control;

    ros::spin();
}
