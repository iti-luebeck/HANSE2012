#include "hanse_gamepad/teleop_joy.h"

TeleopHanse::TeleopHanse():
    linearValue(0),
    angularValue(0),
    depthValue(0),
    depthLastValue(0),
    motorsEnabled(true),
    pidsEnabled(true),
    emergencyStop(false),
    gamepadEnabled(false),
    depthUpLast(false),
    depthDownLast(false)
{

    pubCmdVel = node.advertise<geometry_msgs::Twist>("/hanse/commands/cmd_vel_joystick", 1);
    subJoyInput = node.subscribe<sensor_msgs::Joy>("/hanse/joy", 10, &TeleopHanse::joyCallback, this);

    // will be set to actual value once config is loaded
    publishTimer = node.createTimer(ros::Duration(1), &TeleopHanse::timerCallback, this);

    ros::Rate r(10);

    trig = new FlankTrigger();

    while(ros::Time::now().toSec() == 0.0) {
        r.sleep();
    }

    ignoreTime = ros::Time::now() + ros::Duration(0.25);

    dynReconfigureCb = boost::bind(&TeleopHanse::dynReconfigureCallback, this, _1, _2);
    dynReconfigureSrv.setCallback(dynReconfigureCb);
    // from this point on we can assume a valid config

    srvClEngineCommandDepth = node.serviceClient<hanse_srvs::EngineCommand>("/hanse/engine/depth/handleEngineCommand");
    srvClEngineCommandOrientation = node.serviceClient<hanse_srvs::EngineCommand>("/hanse/engine/orientation/handleEngineCommand");
    srvClEngineSetDepth = node.serviceClient<hanse_srvs::SetTarget>("/hanse/engine/depth/setDepth");
    srvClCmdVelMuxSelect = node.serviceClient<topic_tools::MuxSelect>("/hanse/commands/cmd_vel_select");

    ROS_INFO("teleop_joy started");
    ROS_INFO("Gamepad disabled");
}

void TeleopHanse::dynReconfigureCallback(hanse_gamepad::GamepadNodeConfig &config, uint32_t level) {

    ROS_INFO("got new parameters, level=%d", level);

    this->config = config;

    publishTimer.setPeriod(ros::Duration(1.0/config.publish_frequency));
}

void TeleopHanse::timerCallback(const ros::TimerEvent &e) {

    if (gamepadEnabled && (ros::Time::now() < ignoreTime)) {
        // publish data on topic.
        geometry_msgs::Twist velocityMsg;

        velocityMsg.angular.z = angularValue;

        velocityMsg.linear.x = linearValue;

        // ROS_INFO("Current target depth: %f cm - FF %f - ANG %f - ORI %f", depthValue, linearValue, rotationSpeedValue, orientationValue);

        pubCmdVel.publish(velocityMsg);
    }
}

void TeleopHanse::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    int8_t count = 0;
    ros::Rate loopRate(4);

    if (ros::Time::now() < ignoreTime) {
        return;
    }

    trig->setActual(joy->buttons);

    hanse_srvs::EngineCommand commandMsg;
    commandMsg.request.enableDepthPid = true;
    commandMsg.request.enableMotors = motorsEnabled;
    commandMsg.request.enableOrientationPid = true;
    commandMsg.request.enableRotationSpeedPid = false;
    commandMsg.request.resetZeroPressure = false;
    commandMsg.request.setEmergencyStop = false;
    bool changed = false;

    if (trig->isSet(config.emergency_stop_button)) {
        changed = true;
        emergencyStop = !emergencyStop;
        commandMsg.request.setEmergencyStop = emergencyStop;

        if(emergencyStop) {
            ROS_INFO("Emergency stop! Gamepad disabled, depth is set to 0");
            gamepadEnabled = false;
            depthValue = 0;
            angularValue = 0;
            linearValue = 0;
        } else {
            ROS_INFO("Emergency stop disabled");
        }
    }

    if (trig->isSet(config.motor_switch_button) && !emergencyStop) {
        changed = true;
        motorsEnabled = !motorsEnabled;
        commandMsg.request.enableMotors = motorsEnabled;

        if(motorsEnabled) {
            ROS_INFO("Motors enabled.");
        } else {
            ROS_INFO("Motors disabled.");
        }
    }

    if (trig->isSet(config.pid_switch_button) && !emergencyStop) {
        changed = true;
        pidsEnabled = !pidsEnabled;
        commandMsg.request.enableDepthPid = pidsEnabled;

        if (pidsEnabled) {
            ROS_INFO("PIDs enabled.");
            commandMsg.request.enableOrientationPid = true;
        } else {
            ROS_INFO("PIDs disabled.");
            commandMsg.request.enableOrientationPid = false;
        }
    }

    if (trig->isSet(config.zero_depth_reset_button) && !emergencyStop) {
        changed = true;
        commandMsg.request.resetZeroPressure = true;
        ROS_INFO("Resetted pressure zero value.");
    }

    if (trig->isSet(config.gamepad_switch_button) && !emergencyStop) {
        topic_tools::MuxSelect selectMsg;

        if(!gamepadEnabled) {
            selectMsg.request.topic = "/hanse/commands/cmd_vel_joystick";

            // Aktivierung Gamepad
            while(ros::ok() && count < NUM_SERVICE_LOOPS) {
                if (srvClCmdVelMuxSelect.call(selectMsg)) {
                    count = 0;
                    gamepadEnabled = !gamepadEnabled;
                    ROS_INFO("Gamepad enabled.");
                    break;
                } else {
                    ROS_INFO("Command velocity mux couldn't be called. Retry.");
                }

                count++;
                loopRate.sleep();
            }
        } else {
            selectMsg.request.topic = "/hanse/commands/cmd_vel_behaviour";

            // Deaktivierung Gamepad
            while(ros::ok() && count < NUM_SERVICE_LOOPS) {
                if (srvClCmdVelMuxSelect.call(selectMsg)) {
                    count = 0;
                    gamepadEnabled = !gamepadEnabled;
                    ROS_INFO("Gamepad disabled.");
                    break;
                } else {
                    ROS_INFO("Command velocity mux couldn't be called. Retry.");
                }

                count++;
                loopRate.sleep();
            }
        }
    }

    if (gamepadEnabled) {
        if (fabs(joy->axes[config.linear_axis]) < config.joy_deadzone) {
            linearValue = 0;
        } else {
            linearValue = config.linear_scale * joy->axes[config.linear_axis];
        }

        if (fabs(joy->axes[config.angular_axis]) < config.joy_deadzone) {
            angularValue = 0;
        } else {
            angularValue = config.angular_scale * joy->axes[config.angular_axis];
        }

        if (trig->isSet(config.depth_up_button)) {
            depthValue -= config.depth_delta;
        }

        if (trig->isSet(config.depth_down_button)) {
            depthValue += config.depth_delta;
        }

        if (depthValue < 0) {
            depthValue = 0;
        }

        if (depthValue != depthLastValue) {
            hanse_srvs::SetTarget depthMsg;
            depthMsg.request.target = depthValue;
            depthLastValue = depthValue;

            // Setzen der Tiefe
            while(ros::ok() && count < NUM_SERVICE_LOOPS) {
                if (srvClEngineSetDepth.call(depthMsg)) {
                    count = 0;
                    break;
                } else {
                    ROS_INFO("Target depth couldn't be set. Retry.");
                }

                count++;
                loopRate.sleep();
            }
        }
    }

    if (changed) {
        // Senden der Engine-Nachrichten
        while(ros::ok() && count < NUM_SERVICE_LOOPS) {
            if (srvClEngineCommandDepth.call(commandMsg)) {
                count = 0;
                break;
            } else {
                ROS_INFO("Depth engine couldn't be called. Retry.");
            }

            count++;
            loopRate.sleep();
        }

        while(ros::ok() && count < NUM_SERVICE_LOOPS) {
            if (srvClEngineCommandOrientation.call(commandMsg)) {
                count = 0;
                break;
            } else {
                ROS_INFO("Orientation engine couldn't be called. Retry.");
            }

            count++;
            loopRate.sleep();
        }
    }
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "teleop_hanse");
    ros::start();

    TeleopHanse teleop_hanse;

    ros::spin();
}
