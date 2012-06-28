#include "hanse_gamepad/teleop_joy.h"

TeleopHanse::TeleopHanse():
    linearValue(0),
    angularValue(0),
    depthValue(0),
    motorsEnabled(true),
    pidsEnabled(true),
    emergencyStop(false),
    gamepadEnabled(false) {

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
//    orientationDelta = DEG2RAD(config.orientation_delta);
}

void TeleopHanse::timerCallback(const ros::TimerEvent &e) {

    if (gamepadEnabled) {
        // publish data on topic.
        geometry_msgs::Twist velocityMsg;

//        if (orientationMode) {
//            velocityMsg.angular.z = orientationValue;
//        } else {
            velocityMsg.angular.z = angularValue;
//        }

        velocityMsg.linear.x = linearValue;
        velocityMsg.linear.z = depthValue;

        // ROS_INFO("Current target depth: %f cm - FF %f - ANG %f - ORI %f", depthValue, linearValue, rotationSpeedValue, orientationValue);

        pubCmdVel.publish(velocityMsg);
    }
}

void TeleopHanse::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    hanse_srvs::EngineCommand commandMsg;
    commandMsg.request.enableDepthPid = true;
    commandMsg.request.enableMotors = motorsEnabled;
//    commandMsg.request.enableOrientationPid = orientationMode;
    commandMsg.request.enableOrientationPid = true;
//    commandMsg.request.enableRotationSpeedPid = rotationSpeedMode;
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

//            if (orientationMode) {
                commandMsg.request.enableOrientationPid = true;
//            } else if (rotationSpeedMode) {
//                commandMsg.request.enableRotationSpeedPid = true;
//            }
        } else {
            ROS_INFO("PIDs disabled.");
            commandMsg.request.enableOrientationPid = false;
//            commandMsg.request.enableRotationSpeedPid = false;
        }
    }

//    if (joy->buttons[config.rotation_speed_mode] && !orientationMode && !emergencyStop) {
//        changed = true;
//        rotationSpeedMode = !rotationSpeedMode;
//        commandMsg.request.enableRotationSpeedPid = rotationSpeedMode;

//        if (rotationSpeedMode) {
//            ROS_INFO("Rotation speed control activated.");
//        } else {
//            angularValue = 0;
//            ROS_INFO("Rotation speed control deactivated.");
//        }
//    }

//    if (joy->buttons[config.orientation_mode] && !rotationSpeedMode && !emergencyStop) {
//        changed = true;
//        orientationMode = !orientationMode;
//        commandMsg.request.enableOrientationPid = orientationMode;

//        if (orientationMode) {
//            ROS_INFO("Orientation control activated.");
//        } else {
//            ROS_INFO("Orientation control deactivated.");
//        }
//    }

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

//        if (orientationMode) {
//            if (joy->buttons[config.orientation_left_button]) {
//                orientationValue += orientationDelta;

//                if (orientationValue > M_PI) {
//                    orientationValue -= 2 * M_PI;
//                }
//            } else if (joy->buttons[config.orientation_right_button]) {
//                orientationValue -= orientationDelta;

//                if (orientationValue < -M_PI) {
//                    orientationValue += 2 * M_PI;
//                }
//            }
//        } else {
            if (fabs(joy->axes[config.angular_axis]) < config.joy_deadzone) {
                angularValue = 0;
            } else {
                angularValue = config.angular_scale * joy->axes[config.angular_axis];
            }
//        }

        if (config.depth_down_button == config.depth_down_button) {
            if (joy->axes[config.depth_down_button] != 0) {
                depthValue -= joy->axes[config.depth_down_button] * config.depth_delta;
            }
        } else {
            if (joy->buttons[config.depth_up_button]) {
                depthValue -= config.depth_delta;
            }

            if (joy->buttons[config.depth_down_button]) {
                depthValue += config.depth_delta;
            }
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

