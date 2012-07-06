#include "hanse_gamepad/teleop_joy.h"

TeleopHanse::TeleopHanse():
    linearValue(0),
    angularValue(0),
    depthValue(0),
    motorsEnabled(true),
    pidsEnabled(true),
    emergencyStop(false),
    gamepadEnabled(false),
    depthUpLast(false),
    depthDownLast(false)
{

    pubCmdVel = node.advertise<geometry_msgs::Twist>("/hanse/commands/cmd_vel", 1);
    subJoyInput = node.subscribe<sensor_msgs::Joy>("/hanse/joy", 10, &TeleopHanse::joyCallback, this);

    // will be set to actual value once config is loaded
    publishTimer = node.createTimer(ros::Duration(1), &TeleopHanse::timerCallback, this);

    dynReconfigureCb = boost::bind(&TeleopHanse::dynReconfigureCallback, this, _1, _2);
    dynReconfigureSrv.setCallback(dynReconfigureCb);
    // from this point on we can assume a valid config

    srvClEngineCommandDepth = node.serviceClient<hanse_srvs::EngineCommand>("/hanse/engine/depth/handleEngineCommand");
    srvClEngineCommandOrientation = node.serviceClient<hanse_srvs::EngineCommand>("/hanse/engine/orientation/handleEngineCommand");

    ROS_INFO("teleop_joy started");
    ROS_INFO("Gamepad disabled");
}

void TeleopHanse::dynReconfigureCallback(hanse_gamepad::GamepadNodeConfig &config, uint32_t level) {

    ROS_INFO("got new parameters, level=%d", level);

    this->config = config;

    publishTimer.setPeriod(ros::Duration(1.0/config.publish_frequency));
}

void TeleopHanse::timerCallback(const ros::TimerEvent &e) {

    if (gamepadEnabled) {
        // publish data on topic.
        geometry_msgs::Twist velocityMsg;

        velocityMsg.angular.z = angularValue;

        velocityMsg.linear.x = linearValue;
        velocityMsg.linear.z = depthValue;

        // ROS_INFO("Current target depth: %f cm - FF %f - ANG %f - ORI %f", depthValue, linearValue, rotationSpeedValue, orientationValue);

        pubCmdVel.publish(velocityMsg);
    }
}

void TeleopHanse::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    bool depthUpRisingFlank;
    bool depthDownRisingFlank;

    if (config.depth_up_button == config.depth_down_button) {
        depthUpRisingFlank = !depthUpLast && (joy->axes[config.depth_up_button] == -1);
        depthDownRisingFlank = !depthDownLast && (joy->axes[config.depth_down_button == 1]);
        depthUpLast = joy->axes[config.depth_up_button == -1];
        depthDownLast = joy->axes[config.depth_down_button == 1];
    } else {
        depthUpRisingFlank = !depthUpLast && joy->buttons[config.depth_up_button];
        depthDownRisingFlank = !depthDownLast && joy->buttons[config.depth_down_button];
        depthUpLast = joy->buttons[config.depth_up_button];
        depthDownLast = joy->buttons[config.depth_down_button];
    }

    hanse_srvs::EngineCommand commandMsg;
    commandMsg.request.enableDepthPid = true;
    commandMsg.request.enableMotors = motorsEnabled;
    commandMsg.request.enableOrientationPid = true;
    commandMsg.request.enableRotationSpeedPid = false;
    commandMsg.request.resetZeroPressure = false;
    commandMsg.request.setEmergencyStop = false;
    bool changed = false;

    if (joy->buttons[config.emergency_stop_button]) {
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

    if (joy->buttons[config.motor_switch_button] && !emergencyStop) {
        changed = true;
        motorsEnabled = !motorsEnabled;
        commandMsg.request.enableMotors = motorsEnabled;

        if(motorsEnabled) {
            ROS_INFO("Motors enabled.");
        } else {
            ROS_INFO("Motors disabled.");
        }
    }

    if (joy->buttons[config.pid_switch_button] && !emergencyStop) {
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

    if (joy->buttons[config.zero_depth_reset_button] && !emergencyStop) {
        changed = true;
        commandMsg.request.resetZeroPressure = true;
    }

    if (joy->buttons[config.gamepad_switch_button] && !emergencyStop) {
        gamepadEnabled = !gamepadEnabled;

        if(gamepadEnabled) {
            ROS_INFO("Gamepad enabled.");
        } else {
            ROS_INFO("Gamepad disabled.");
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

        if (depthUpRisingFlank) {
            depthValue += config.depth_delta;
        }

        if (depthDownRisingFlank) {
            depthValue -= config.depth_delta;
        }

        if (depthValue < 0) {
            depthValue = 0;
        }
    }

    if (changed) {
        int8_t count = 0;
        // Senden der Engine-Nachricht
        ros::Rate loopRate(4);
        while(ros::ok() && count < NUM_SERVICE_LOOPS) {
            if (srvClEngineCommandDepth.call(commandMsg) &&
                    srvClEngineCommandOrientation.call(commandMsg)) {
                break;
            } else {
                ROS_INFO("Engine couldn't be called. Retry.");
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

