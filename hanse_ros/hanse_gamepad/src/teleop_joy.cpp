#include "hanse_gamepad/teleop_joy.h"

TeleopHanse::TeleopHanse() :
    linear_value_(0),
    angular_value_(0),
    motors_enabled_(true),
    pids_enabled_(true),
    emergency_stop_(false),
    gamepad_enabled_(false),
    gamepad_delay_ended_(false),
    trig_(),
    depth_cmd_h_("/hanse/engine/depth/handleEngineCommand"),
    orientation_cmd_h_("/hanse/engine/orientation/handleEngineCommand")
{
    // Publisher for command velocity messages
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(
                "/hanse/commands/cmd_vel_joystick", 1);
    // Subscriber for joystick events
    sub_joy_input_ = nh_.subscribe<sensor_msgs::Joy>(
                "/hanse/joy", 10, &TeleopHanse::joyCallback, this);

    // will be set to actual value once config is loaded
    publish_timer_ = nh_.createTimer(
                ros::Duration(0.1), &TeleopHanse::timerCallback, this);

    delay_timer_ = nh_.createTimer(
                ros::Duration(0.5), &TeleopHanse::delayCallback, this, true, false);

    // initialize dynamic reconfigure
    dyn_reconfigure_cb_ = boost::bind(
                &TeleopHanse::dynReconfigureCallback, this, _1, _2);
    dyn_reconfigure_srv_.setCallback(dyn_reconfigure_cb_);
    // from this point on we can assume a valid config

    // service for setting depth in depth engine
    srv_client_engine_inc_depth_ = nh_.serviceClient<hanse_srvs::SetTarget>(
                "/hanse/engine/depth/incDepth");
    // service for muxing command velocity messages
    srv_vel_mux_select_ = nh_.serviceClient<topic_tools::MuxSelect>(
                "/hanse/commands/cmd_vel_mux/select");

    delay_timer_.start();
    ROS_INFO("teleop_joy started");
    ROS_INFO("Gamepad disabled");
}

void TeleopHanse::dynReconfigureCallback(hanse_gamepad::GamepadNodeConfig &config,
                                         uint32_t level)
{
    ROS_INFO("got new parameters, level=%d", level);
    config_ = config;

    // refresh publish timer frequency
    publish_timer_.setPeriod(ros::Duration(1.0/config.publish_frequency));
}

void TeleopHanse::timerCallback(const ros::TimerEvent &e)
{
    // only send messages if gamepad is enabled and delay is over
    if (gamepad_enabled_ && gamepad_delay_ended_)
    {
        // publish data on topic.
        geometry_msgs::Twist velocity_msg;
        velocity_msg.angular.z = angular_value_;
        velocity_msg.linear.x = linear_value_;

        pub_cmd_vel_.publish(velocity_msg);
    }
}

void TeleopHanse::delayCallback(const ros::TimerEvent &e)
{
    gamepad_delay_ended_ = true;
    ROS_INFO("Initial delay ended.");
}

void TeleopHanse::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // ignore initial events
    if (!gamepad_delay_ended_)
    {
        return;
    }

    // do rising flank triggering of buttons
    trig_.setActual(joy->buttons);

    // handle emergency stop
    if (trig_.isSet(config_.emergency_stop_button) &&
            !EngineCommandHandle::isEmergencyStopChanged())
    {
        EngineCommandHandle::toggleEmergencyStop();
    }

    // handle motor switch button
    if (trig_.isSet(config_.motor_switch_button) &&
            !EngineCommandHandle::isEmergencyStopSet())
    {
        EngineCommandHandle::toggleMotors();
    }

    // handle resetting zero pressure
    if (trig_.isSet(config_.zero_depth_reset_button) &&
            !EngineCommandHandle::isEmergencyStopSet())
    {
        EngineCommandHandle::resetZeroPressure();
        ROS_INFO("Resetted pressure zero value.");
    }

    // handle gamepad enabling and disabling
    if (trig_.isSet(config_.gamepad_switch_button) &&
            !EngineCommandHandle::isEmergencyStopSet())
    {
        if(!gamepad_enabled_)
        {
            if (switchGamepadUsage("/hanse/commands/cmd_vel_joystick"))
            {
                ROS_INFO("Gamepad enabled.");
            }
            else
            {
                ROS_INFO("Command velocity mux couldn't be called.");
            }
        }
        else
        {
            if (switchGamepadUsage("/hanse/commands/cmd_vel_behaviour"))
            {
                ROS_INFO("Gamepad disabled.");
            }
            else
            {
                ROS_INFO("Command velocity mux couldn't be called.");
            }
        }
    }

    // handle movement if gamepad enabled
    if (gamepad_enabled_)
    {
        handleGamepadMove(joy->axes);
    }

    // send engine command to depth and orientation engine
    sendEngineCommands();
}

bool TeleopHanse::switchGamepadUsage(const char *commandVelocityTopic)
{
    uint8_t counter = 0;
    ros::Rate loop_rate(4);

    // select message for command velocity muxing
    topic_tools::MuxSelect select_msg;
    select_msg.request.topic = commandVelocityTopic;

    // loop and try to call the command velocity mux to switch input
    while(ros::ok() && counter < NUM_SERVICE_LOOPS)
    {
        if (srv_vel_mux_select_.call(select_msg))
        {
            gamepad_enabled_ = !gamepad_enabled_;
            return true;
        }

        counter++;
        loop_rate.sleep();
    }

    return false;
}

void TeleopHanse::handleGamepadMove(std::vector<float> axes)
{
    double depth = 0;

    // handle linear movement
    if (fabs(axes[config_.linear_axis]) < config_.joy_deadzone)
    {
        linear_value_ = 0;
    }
    else
    {
        linear_value_ = config_.linear_scale * axes[config_.linear_axis];
    }

    // handle angular movement
    if (fabs(axes[config_.angular_axis]) < config_.joy_deadzone)
    {
        angular_value_ = 0;
    }
    else
    {
        angular_value_ = config_.angular_scale * axes[config_.angular_axis];
    }

    // handle going up
    if (trig_.isSet(config_.depth_up_button))
    {
        depth = -config_.depth_delta;
    }

    // handle going deeper
    if (trig_.isSet(config_.depth_down_button))
    {
        depth = config_.depth_delta;
    }

    // call depth engine to set the target depth
    if (depth != 0.0)
    {
        sendTargetDepth(depth);
    }
}

bool TeleopHanse::sendTargetDepth(const double increment)
{
    uint8_t counter = 0;
    ros::Rate loop_rate(4);

    hanse_srvs::SetTarget depth_msg;
    depth_msg.request.target = increment;

    // Setzen der Tiefe
    while(ros::ok() && counter < NUM_SERVICE_LOOPS)
    {
        if (srv_client_engine_inc_depth_.call(depth_msg))
        {
            return true;
        }

        counter++;
        loop_rate.sleep();
    }

    return false;
}

void TeleopHanse::sendEngineCommands()
{   
    // send depth and orientation command message
    bool depth_sent = depth_cmd_h_.sendMsg();
    bool sent = depth_sent && orientation_cmd_h_.sendMsg();

    // if depth could be sent and was emergency stop, print changes and
    // setting target depth to 0
    if (depth_sent)
    {
        if (EngineCommandHandle::isEmergencyStopSet())
        {
            EngineCommandHandle::printChanges();

            ROS_INFO("Depth is set to 0.");
        }
    }
    // reset message if it couldn't be send
    else
    {
        EngineCommandHandle::recoverState();
        ROS_INFO("Depth engine couldn't be called.");
    }

    // if both messages could be sent and was emergency stop, disable
    // movement and gamepad
    if (sent)
    {
        if (EngineCommandHandle::isEmergencyStopSet())
        {
            angular_value_ = 0;
            linear_value_ = 0;
            ROS_INFO("Movement is stopped.");
            gamepad_enabled_ = false;
            ROS_INFO("Gamepad disabled.");
        }

        // print changes and reset message
        EngineCommandHandle::printChanges();
        EngineCommandHandle::resetMsg();
    }
    // if orientation message could not be send, and was not emergency stop,
    // reset message to previous state
    // if emergency stop, send this message again next time until both calls
    // succeed or none of them
    else
    {
        if (!EngineCommandHandle::isEmergencyStopSet())
        {
            EngineCommandHandle::recoverState();
        }

        ROS_INFO("Orientation engine couldn't be called.");
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "teleop_hanse");
    ros::start();

    TeleopHanse teleop_hanse;

    ros::spin();
}
