#include <cmath>

#include "hanse_orientationengine/orientation_engine.h"

OrientationEngine::OrientationEngine() :
    linearSpeed(0),
    angularSpeed(0),
    orientationCurrent(0),
    orientationTarget(0),
    orientationInit(false),
    emergencyStop(false),
    pidsEnabled(true),
    orientationPidEnabled(false),
    motorsEnabled(true),
    orientationOutput(0)
{
    // Initialisierung der Standard-Service Nachrichten.
    enableMsg.request.enable = true;
    disableMsg.request.enable = false;

    // Registrierung der Publisher und Subscriber.
    pubOrientationCurrent = node.advertise<std_msgs::Float64>("/hanse/pid/orientation/input", 1);

    pubMotorLeft = node.advertise<hanse_msgs::sollSpeed>("/hanse/motors/left", 1);
    pubMotorRight = node.advertise<hanse_msgs::sollSpeed>("/hanse/motors/right", 1);

    subVelocity = node.subscribe<geometry_msgs::Twist>("/hanse/commands/cmd_vel", 10,
                                                       &OrientationEngine::velocityCallback, this);
    subXsens = node.subscribe<sensor_msgs::Imu>(
                "/hanse/imu", 10, &OrientationEngine::xsensCallback, this);

    subOrientationOutput = node.subscribe<std_msgs::Float64>(
                "/hanse/pid/orientation/output", 10, &OrientationEngine::orientationOutputCallback, this);

    publishTimer = node.createTimer(ros::Duration(1),
                                    &OrientationEngine::publishTimerCallback, this);

    // Registrierung der Services.
    srvHandleEngineCommand = node.advertiseService("engine/orientation/handleEngineCommand",
                                                   &OrientationEngine::handleEngineCommand, this);

    srvEnableOrientationPid = node.advertiseService("engine/orientation/enableOrientationPid",
                                                    &OrientationEngine::enableOrientationPid, this);

    srvEnableMotors = node.advertiseService("engine/orientation/enableMotors",
                                            &OrientationEngine::enableMotors, this);

    srvSetEmergencyStop = node.advertiseService("engine/orientation/setEmergencyStop",
                                                &OrientationEngine::setEmergencyStop, this);

    dynReconfigureCb = boost::bind(&OrientationEngine::dynReconfigureCallback, this, _1, _2);
    dynReconfigureSrv.setCallback(dynReconfigureCb);

    // Registrierung der Service Clients.
    srvClOrientationPid  = node.serviceClient<hanse_srvs::Bool>("/hanse/pid/orientation/enable");

    if (config.orientation_pid_enabled_at_start) {
        if (callOrientationPidEnableService(true)) {
            ROS_INFO("Orientation PID enabled.");
            orientationPidEnabled = true;
        } else {
            ROS_ERROR("Orientation PID couldn't be enabled. Shutdown.");
            ros::shutdown();
        }
    }

    ROS_INFO("Orientation engine started.");
}

void OrientationEngine::dynReconfigureCallback(hanse_orientationengine::OrientationengineConfig &config, uint32_t level) {
    ROS_INFO("got new parameters, level=%d", level);

    this->config = config;

    publishTimer.setPeriod(ros::Duration(1.0/config.publish_frequency));
}

// Auswertung und Zwischenspeicherung der cmd_vel Nachricht.
void OrientationEngine::velocityCallback(
        const geometry_msgs::Twist::ConstPtr& twist) {

    linearSpeed = twist->linear.x;
    angularSpeed = twist->angular.z;

    if (angularSpeed == 0 && !orientationPidEnabled) {
        callOrientationPidEnableService(true);
        orientationPidEnabled = true;
    } else if (angularSpeed != 0 && orientationPidEnabled) {
        callOrientationPidEnableService(false);
        orientationPidEnabled = false;
    }
}

// Auswertung und Zwischenspeicherung der XSens Eingabedaten.
void OrientationEngine::xsensCallback(
        const sensor_msgs::Imu::ConstPtr& xsensData) {

    Eigen::Quaternionf flipped(xsensData->orientation.w,
                               xsensData->orientation.x,
                               xsensData->orientation.y,
                               xsensData->orientation.z);

    // This undos the rotation of the xsens (180 deg around the y
    // axis) this is why: we first (right side) put the xsens into hanse the
    // wrong way around and _then_ move hanse around (left side) so
    // the inverse of the xsens orientation has to be at the right
    // side too for them to cancle
    Eigen::Quaternionf orientation = flipped * Eigen::AngleAxis<float>(M_PI, Eigen::Vector3f(0, 1, 0));
    Eigen::Vector3f direction = orientation * Eigen::Vector3f(1, 0, 0);

    orientationCurrent = atan2(direction.y(), direction.x());

    if (angularSpeed != 0 || !orientationInit) {
        orientationInit = true;
        orientationTarget = orientationCurrent;
    }
}

// Auswertung und Zwischenspeicherung der Ausgabedaten des Orientierungs-PID-Reglers.
void OrientationEngine::orientationOutputCallback(
        const std_msgs::Float64::ConstPtr& orientationOutput) {

    this->orientationOutput = orientationOutput->data;
}

// Ausführung jeweils nach Ablauf des Timers. Wird verwendet, um sämtliche
// Ausgaben auf die vorgesehenen Topics zu schreiben.
void OrientationEngine::publishTimerCallback(const ros::TimerEvent &e) {

    hanse_msgs::sollSpeed motorLeftMsg;
    hanse_msgs::sollSpeed motorRightMsg;
    motorLeftMsg.data = 0;
    motorRightMsg.data = 0;

    if(!emergencyStop && motorsEnabled) {
         // Orientierungssteuerung.
         std_msgs::Float64 orientationMsg;
         double rotation;
         //double absoluteDistance = std::abs(orientationTarget) + std::abs(orientationCurrent);

         rotation = orientationTarget - orientationCurrent;

         if(rotation < -M_PI) {
             rotation = rotation + 2 * M_PI;
         } else if (rotation >= M_PI){
             rotation = rotation - 2 * M_PI;
         }

         orientationMsg.data = rotation;
         pubOrientationCurrent.publish(orientationMsg);

         // Motorsteuerung
         int16_t motorLeft = 0;
         int16_t motorRight = 0;

         // Berechnung der Ansteuerungsstärke der seitlichen Motoren.
         if (angularSpeed == 0) {
             motorLeft = linearSpeed * 127 - orientationOutput;
             motorRight = linearSpeed * 127 + orientationOutput;
         } else {
             motorLeft = linearSpeed * 127 - angularSpeed * 127;
             motorRight = linearSpeed * 127 + angularSpeed * 127;
         }

         // Beschränkung der Ansteuerungsstärke auf -127 bis 127.
         if(motorLeft > 127) {
             motorLeft = 127;
         } else if(motorLeft < -127) {
             motorLeft = -127;
         }

         if(motorRight > 127) {
             motorRight = 127;
         } else if(motorRight < -127) {
             motorRight = -127;
         }

         motorLeftMsg.data = motorLeft;
         motorRightMsg.data = motorRight;
    }

    pubMotorLeft.publish(motorLeftMsg);
    pubMotorRight.publish(motorRightMsg);
}

bool OrientationEngine::handleEngineCommand(hanse_srvs::EngineCommand::Request &req,
                                        hanse_srvs::EngineCommand::Response &res) {

    if (req.setEmergencyStop && !emergencyStop) {
        emergencyStop = true;
        pidsEnabled = false;

        // Deaktivierung aller PID-Regler.
        if(orientationPidEnabled) {
            if (callOrientationPidEnableService(false)) {
                ROS_INFO("Orientation PID disabled.");
            } else {
                ROS_ERROR("Orientation PID couldn't be disabled.");
            }
        }

        // Bewegung auf 0 setzen
        linearSpeed = 0;
        angularSpeed = 0;
    } else if (!req.setEmergencyStop) {
        if (emergencyStop) {
            emergencyStop = false;
            pidsEnabled = true;

            if (req.enableOrientationPid) {
                if (callOrientationPidEnableService(true)) {
                    ROS_INFO("Orientation PID enabled.");
                    orientationPidEnabled = true;
                } else {
                    ROS_ERROR("Orientation PID couldn't be enabled.");
                    orientationPidEnabled = false;
                }
            }
        } else {
            if (!orientationPidEnabled && req.enableOrientationPid) {
                if (callOrientationPidEnableService(true)) {
                    ROS_INFO("Orientation PID enabled.");
                    orientationPidEnabled = true;
                } else {
                    ROS_ERROR("Orientation PID couldn't be enabled.");
                }
            } else if (orientationPidEnabled && !req.enableOrientationPid) {
                if (callOrientationPidEnableService(false)) {
                    ROS_INFO("Orientation PID disabled.");
                    orientationPidEnabled = false;
                } else {
                    ROS_ERROR("Orientation PID couldn't be disabled.");
                }
            }
        }

        if (motorsEnabled != req.enableMotors) {
            if (req.enableMotors) {
                ROS_INFO("Motors enabled.");
            } else {
                ROS_INFO("Motors disabled.");
            }

            motorsEnabled = req.enableMotors;
        }
    }

    return true;
}

bool OrientationEngine::enableOrientationPid(hanse_srvs::Bool::Request &req,
                                         hanse_srvs::Bool::Response &res) {

    if (pidsEnabled) {
        if (!orientationPidEnabled && req.enable) {
            if (callOrientationPidEnableService(true)) {
                ROS_INFO("Orientation PID enabled.");
                orientationPidEnabled = true;
            } else {
                ROS_ERROR("Orientation PID couldn't be enabled.");
            }
        } else if (orientationPidEnabled && !req.enable) {
            if (callOrientationPidEnableService(false)) {
                ROS_INFO("Orientation PID disabled.");
                orientationPidEnabled = false;
            } else {
                ROS_ERROR("Orientation PID couldn't be disabled.");
            }
        }
    } else {
        orientationPidEnabled = req.enable;
    }

    return true;
}

bool OrientationEngine::enableMotors(hanse_srvs::Bool::Request &req,
                                 hanse_srvs::Bool::Response &res) {
    motorsEnabled = req.enable;

    return true;
}

bool OrientationEngine::setEmergencyStop(hanse_srvs::Bool::Request &req,
                                     hanse_srvs::Bool::Response &res) {

    if (req.enable && !emergencyStop) {
        emergencyStop = true;
        pidsEnabled = false;

        // Deaktivierung aller PID-Regler.
        if(orientationPidEnabled) {
            if (callOrientationPidEnableService(false)) {
                ROS_INFO("Orientation PID disabled.");
            } else {
                ROS_ERROR("Orientation PID couldn't be disabled.");
            }
        }

        // Bewegung auf 0 setzen
        linearSpeed = 0;
        angularSpeed = 0;
    } else if (!req.enable && emergencyStop) {
        emergencyStop = false;
        pidsEnabled = true;

        if(orientationPidEnabled) {
            if (callOrientationPidEnableService(true)) {
                ROS_INFO("Orientation PID enabled.");
            } else {
                ROS_ERROR("Orientation PID couldn't be enabled.");
            }
        }
    }

    return true;
}

bool OrientationEngine::callOrientationPidEnableService(const bool msg) {
    int8_t count = 0;
    ros::Rate loopRate(4);
    bool success = false;

    while(ros::ok() && count < NUM_SERVICE_LOOPS) {
        if ((msg && srvClOrientationPid.call(enableMsg)) || (!msg && srvClOrientationPid.call(disableMsg))) {
            success = true;
            break;
        }

        count++;
        loopRate.sleep();
    }

    return success;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "engine_control");

    ros::start();

    OrientationEngine engine_control;

    ros::spin();
}
