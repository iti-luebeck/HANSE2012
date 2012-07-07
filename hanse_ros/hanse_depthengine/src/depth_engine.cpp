#include "hanse_depthengine/depth_engine.h"

DepthEngine::DepthEngine() :
    depthTarget(0),
    pressureBias(0),
    pressureCurrent(0),
    pressureInit(false),
    emergencyStop(false),
    pidsEnabled(true),
    depthPidEnabled(true),
    motorsEnabled(true),
    depthOutput(0)
{
    // Initialisierung der Standard-Service Nachrichten.
    enableMsg.request.enable = true;
    disableMsg.request.enable = false;

    // Registrierung der Publisher und Subscriber.
    pubDepthCurrent = node.advertise<std_msgs::Float64>("/hanse/pid/depth/input", 1);
    pubDepthTarget = node.advertise<std_msgs::Float64>("/hanse/pid/depth/target",1);

    pubMotorFront = node.advertise<hanse_msgs::sollSpeed>("/hanse/motors/up", 1);
//    pubMotorRear = node.advertise<hanse_msgs::sollSpeed>("/hanse/motors/downBack", 1);

    subPressure = node.subscribe<hanse_msgs::pressure>("/hanse/pressure/depth", 10,
                                                       &DepthEngine::pressureCallback, this);

    subDepthOutput = node.subscribe<std_msgs::Float64>(
                "/hanse/pid/depth/output", 10, &DepthEngine::depthOutputCallback, this);

    publishTimer = node.createTimer(ros::Duration(1),
                                    &DepthEngine::publishTimerCallback, this);

    // Registrierung der Services.
    srvHandleEngineCommand = node.advertiseService("engine/depth/handleEngineCommand",
                                                   &DepthEngine::handleEngineCommand, this);

    srvEnableDepthPid = node.advertiseService("engine/depth/enableDepthPid",
                                              &DepthEngine::enableDepthPid, this);

    srvEnableMotors = node.advertiseService("engine/depth/enableMotors",
                                            &DepthEngine::enableMotors, this);

    srvResetZeroPressure = node.advertiseService("engine/depth/resetZeroPressure",
                                                 &DepthEngine::resetZeroPressure, this);

    srvSetEmergencyStop = node.advertiseService("engine/depth/setEmergencyStop",
                                                &DepthEngine::setEmergencyStop, this);

    srvSetDepth = node.advertiseService("engine/depth/setDepth", &DepthEngine::setDepth, this);

    dynReconfigureCb = boost::bind(&DepthEngine::dynReconfigureCallback, this, _1, _2);
    dynReconfigureSrv.setCallback(dynReconfigureCb);

    // Registrierung der Service Clients.
    srvClDepthPid  = node.serviceClient<hanse_srvs::Bool>("/hanse/pid/depth/enable");

    // Aktivierung des Tiefen-PID-Regler.
    if (config.depth_pid_enabled_at_start) {
        if (callDepthPidEnableService(true)) {
            ROS_INFO("Depth PID enabled.");
            depthPidEnabled = true;
        } else {
            ROS_ERROR("Depth PID couldn't be enabled. Shutdown.");
            ros::shutdown();
        }
    }

    ROS_INFO("Depth engine started.");
}

void DepthEngine::dynReconfigureCallback(hanse_depthengine::DepthengineConfig &config, uint32_t level) {
    ROS_INFO("got new parameters, level=%d", level);

    this->config = config;

    publishTimer.setPeriod(ros::Duration(1.0/config.publish_frequency));
}

// Auswertung und Zwischenspeicherung der Eingabedaten des Drucksensors.
void DepthEngine::pressureCallback(
        const hanse_msgs::pressure::ConstPtr& pressure) {

    if (!pressureInit) {
        pressureBias = pressure->data;
        pressureInit = true;
    }

    pressureCurrent = pressure->data;
}

// Speicherung der Zieltiefe
bool DepthEngine::setDepth(
        hanse_srvs::SetTarget::Request &req,
        hanse_srvs::SetTarget::Response &res) {

    depthTarget = req.target;
    return true;
}

// Auswertung und Zwischenspeicherung der Ausgabedaten des Druck-PID-Reglers.
void DepthEngine::depthOutputCallback(
        const std_msgs::Float64::ConstPtr& depthOutput) {

    this->depthOutput = depthOutput->data;
}

// Ausführung jeweils nach Ablauf des Timers. Wird verwendet, um sämtliche
// Ausgaben auf die vorgesehenen Topics zu schreiben.
void DepthEngine::publishTimerCallback(const ros::TimerEvent &e) {

    hanse_msgs::sollSpeed motorHeightMsg;
    motorHeightMsg.header.stamp = ros::Time::now();

    if(emergencyStop) {
        motorHeightMsg.data = 127;
    } else {
        if (motorsEnabled) {
            // Tiefensteuerung.
            std_msgs::Float64 depthTargetMsg;
            depthTargetMsg.data = depthTarget + pressureBias;

            if (depthTargetMsg.data < config.min_depth_pressure) {
                depthTargetMsg.data = config.min_depth_pressure;
            } else if(depthTargetMsg.data > config.max_depth_pressure) {
                depthTargetMsg.data = config.max_depth_pressure;
            }

            pubDepthTarget.publish(depthTargetMsg);

            std_msgs::Float64 depthCurrentMsg;
            depthCurrentMsg.data = pressureCurrent;

            pubDepthCurrent.publish(depthCurrentMsg);

            motorHeightMsg.data = -depthOutput;
        } else {
            motorHeightMsg.data = 0;
        }
    }

    pubMotorFront.publish(motorHeightMsg);
//    pubMotorRear.publish(motorHeightMsg);
}

bool DepthEngine::handleEngineCommand(hanse_srvs::EngineCommand::Request &req,
                                        hanse_srvs::EngineCommand::Response &res) {

    if (req.setEmergencyStop && !emergencyStop) {
        emergencyStop = true;
        pidsEnabled = false;

        // Deaktivierung des Tiefen-PID-Reglers.
        if(depthPidEnabled) {
            if (callDepthPidEnableService(false)) {
                ROS_INFO("Depth PID disabled.");
            } else {
                ROS_ERROR("Depth PID couldn't be disabled.");
            }
        }

        // Tiefe auf 0 setzen
        depthTarget = 0;
    } else if (!req.setEmergencyStop) {
        if (emergencyStop) {
            emergencyStop = false;
            pidsEnabled = true;

            if (req.enableDepthPid) {
                if (callDepthPidEnableService(true)) {
                    ROS_INFO("Depth PID enabled.");
                } else {
                    ROS_ERROR("Depth PID couldn't be enabled.");
                }
            }

            depthPidEnabled = req.enableDepthPid;
        } else {
            if (!depthPidEnabled && req.enableDepthPid) {
                if (callDepthPidEnableService(true)) {
                    ROS_INFO("Depth PID enabled.");
                    depthPidEnabled = true;
                } else {
                    ROS_ERROR("Depth PID couldn't be enabled.");
                }
            } else if (depthPidEnabled && !req.enableDepthPid) {
                if (callDepthPidEnableService(false)) {
                    ROS_INFO("Depth PID disabled.");
                    depthPidEnabled = false;
                } else {
                    ROS_ERROR("Depth PID couldn't be disabled.");
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

        if (req.resetZeroPressure) {
            ROS_INFO("Resetting zero pressure.");
            pressureInit = false;
        }
    }

    return true;
}

bool DepthEngine::enableDepthPid(hanse_srvs::Bool::Request &req,
                                   hanse_srvs::Bool::Response &res) {

    if (pidsEnabled) {
        if (!depthPidEnabled && req.enable) {
            if (callDepthPidEnableService(true)) {
                ROS_INFO("Depth PID enabled.");
                depthPidEnabled = true;
            } else {
                ROS_ERROR("Depth PID couldn't be enabled.");
            }
        } else if (depthPidEnabled && !req.enable) {
            if (callDepthPidEnableService(false)) {
                ROS_INFO("Depth PID disabled.");
                depthPidEnabled = false;
            } else {
                ROS_ERROR("Depth PID couldn't be disabled.");
            }
        }
    } else {
        depthPidEnabled = req.enable;
    }

    return true;
}

bool DepthEngine::enableMotors(hanse_srvs::Bool::Request &req,
                                 hanse_srvs::Bool::Response &res) {
    motorsEnabled = req.enable;

    return true;
}

bool DepthEngine::resetZeroPressure(hanse_srvs::Empty::Request &req,
                                      hanse_srvs::Empty::Response &res) {
    pressureInit = false;

    return true;
}

bool DepthEngine::setEmergencyStop(hanse_srvs::Bool::Request &req,
                                     hanse_srvs::Bool::Response &res) {

    if (req.enable && !emergencyStop) {
        emergencyStop = true;
        pidsEnabled = false;

        // Deaktivierung des Tiefen-PID-Reglers.
        if(depthPidEnabled) {
            if (callDepthPidEnableService(false)) {
                ROS_INFO("Depth PID disabled.");
            } else {
                ROS_ERROR("Depth PID couldn't be disabled.");
            }
        }

        // Tiefe auf 0 setzen
        depthTarget = 0;
    } else if (!req.enable && emergencyStop) {
        emergencyStop = false;
        pidsEnabled = true;

        // Aktivierung der zuvor aktivierten PID Regler
        if (depthPidEnabled) {
            if (callDepthPidEnableService(true)) {
                ROS_INFO("Depth PID enabled.");
            } else {
                ROS_ERROR("Depth PID couldn't be enabled.");
            }
        }
    }

    return true;
}

bool DepthEngine::callDepthPidEnableService(const bool msg) {
    int8_t count = 0;
    ros::Rate loopRate(4);
    bool success = false;

    while(ros::ok() && count < NUM_SERVICE_LOOPS) {
        if ((msg && srvClDepthPid.call(enableMsg)) || (!msg && srvClDepthPid.call(disableMsg))) {
            success = true;
            break;
        }

        count++;
        loopRate.sleep();
    }

    return success;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depthengine_node");

    ros::start();

    DepthEngine engine_control;

    ros::spin();
}
