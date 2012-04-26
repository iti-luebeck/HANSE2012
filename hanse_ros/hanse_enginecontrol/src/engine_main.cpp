#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <hanse_msgs/pressure.h>
#include <hanse_msgs/sollSpeed.h>
#include <hanse_pidcontrol/GetOutput.h>
#include <hanse_pidcontrol/Enable.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>
#include <tf/tf.h>
#include <boost/thread/mutex.hpp>
#include <cmath>

class EngineControl {
public:
	EngineControl();

private:
	// Node Handle.
	ros::NodeHandle node;

	// Service Clients.
	ros::ServiceClient depthPid;
	ros::ServiceClient orientationPid;
	ros::ServiceClient rotationSpeedPid;

	// Subscriber.
	ros::Subscriber subPressure;
	ros::Subscriber subVelocity;
	ros::Subscriber subXsens;

	ros::Subscriber subSwitchPids;

	ros::Subscriber subDepthOutput;
	ros::Subscriber subOrientationOutput;
	ros::Subscriber subOrientationTarget;
	ros::Subscriber subRotationSpeedOutput;

	// Publisher.
	ros::Publisher pubDepthCurrent;
	ros::Publisher pubDepthTarget;
	ros::Publisher pubOrientationCurrent;
	ros::Publisher pubOrientationTarget;
	ros::Publisher pubRotationSpeedCurrent;
	ros::Publisher pubRotationSpeedTarget;

	ros::Publisher pubMotorFront;
	ros::Publisher pubMotorRear;
	ros::Publisher pubMotorLeft;
	ros::Publisher pubMotorRight;

    // Mutex für parallele Zugriffe
	boost::mutex mutex;

	// Daten Zwischenspeicher.
	double linearSpeedTarget;

	double depthTarget;
	uint16_t pressureBias;
	uint16_t pressureCurrent;
	bool pressureInit;

	double orientationBias;
	double orientationCurrent;
	double orientationTarget;
	double orientationZ;

	double rotationSpeedCurrent;
	double rotationSpeedScale;
	double rotationSpeedTarget;

	// Ausgabewerte der PID-Regler.
	int8_t depthOutput;
	int8_t orientationOutput;
	int8_t rotationSpeedOutput;

	double publishFrequency;

	ros::Timer publishTimer;

	// Methodendeklaration.
	void pressureCallback(const hanse_msgs::pressure::ConstPtr& pressure);
	void velocityCallback(const geometry_msgs::Twist::ConstPtr& twist);
	void xsensCallback(const sensor_msgs::Imu::ConstPtr& xsensData);

	void switchPids(const std_msgs::Bool::ConstPtr& enableRotationSpeedPid);

	void depthOutputCallback(const std_msgs::Float64::ConstPtr& depthOutput);
	void orientationTargetCallback(const std_msgs::Float64::ConstPtr& orientationTarget);
	void orientationOutputCallback(const std_msgs::Float64::ConstPtr& orientationOutput);
	void rotationSpeedOutputCallback(const std_msgs::Float64::ConstPtr& rotationSpeedOutput);

	void publishTimerCallback(const ros::TimerEvent &e);
};

EngineControl::EngineControl() :
            linearSpeedTarget(0),
            depthTarget(0),
            pressureBias(0),
            pressureCurrent(0),
            pressureInit(false),
            orientationCurrent(0),
            orientationTarget(0),
            rotationSpeedCurrent(0),
            rotationSpeedTarget(0),
            depthOutput(0),
            orientationOutput(0),
            rotationSpeedOutput(0)
    {

	// Einlesen der Konfiguration.
	if (!node.getParam("orientation_bias", orientationBias))
		orientationBias = 0.0;

	if (!node.getParam("rotation_speed_scale", rotationSpeedScale))
		rotationSpeedScale = 20.0;

	if (!node.getParam("publish_frequency", publishFrequency))
		publishFrequency = 10.0;

	if (!node.getParam("orientation_z", orientationZ))
		orientationZ = -1.0;

	// Registrierung der Publisher und Subscriber.
	pubDepthCurrent = node.advertise<std_msgs::Float64>("/hanse/pid/depth/input", 1);
	pubDepthTarget = node.advertise<std_msgs::Float64>("/hanse/pid/depth/target",1);
	pubOrientationCurrent = node.advertise<std_msgs::Float64>("/hanse/pid/orientation/input", 1);
	pubOrientationTarget = node.advertise<std_msgs::Float64>("/hanse/pid/orientation/target",1);
	pubRotationSpeedCurrent = node.advertise<std_msgs::Float64>("/hanse/pid/rotationSpeed/input", 1);
	pubRotationSpeedTarget = node.advertise<std_msgs::Float64>("/hanse/pid/rotationSpeed/target",1);

	pubMotorFront = node.advertise<hanse_msgs::sollSpeed>("/hanse/motors/downFront", 1);
	pubMotorRear = node.advertise<hanse_msgs::sollSpeed>("/hanse/motors/downBack", 1);
	pubMotorLeft = node.advertise<hanse_msgs::sollSpeed>("/hanse/motors/left", 1);
	pubMotorRight = node.advertise<hanse_msgs::sollSpeed>("/hanse/motors/right", 1);

	subPressure = node.subscribe<hanse_msgs::pressure>("/hanse/pressure/depth", 10,
			&EngineControl::pressureCallback, this);
	subVelocity = node.subscribe<geometry_msgs::Twist>("/hanse/commands/cmd_vel", 10,
			&EngineControl::velocityCallback, this);
	subXsens = node.subscribe<sensor_msgs::Imu>(
			"/hanse/imu", 10, &EngineControl::xsensCallback, this);

	subSwitchPids = node.subscribe<std_msgs::Bool>("/hanse/commands/switchPid", 10,
			&EngineControl::switchPids, this);

	subDepthOutput = node.subscribe<std_msgs::Float64>(
			"/hanse/pid/depth/output", 10, &EngineControl::depthOutputCallback, this);
	subOrientationOutput = node.subscribe<std_msgs::Float64>(
			"/hanse/pid/orientation/output", 10, &EngineControl::orientationOutputCallback, this);
	subOrientationTarget = node.subscribe<std_msgs::Float64>(
			"/hanse/commands/orientation_target", 10, &EngineControl::orientationTargetCallback, this);
	subRotationSpeedOutput = node.subscribe<std_msgs::Float64>(
			"/hanse/pid/rotationSpeed/output", 10, &EngineControl::rotationSpeedOutputCallback, this);

	publishTimer = node.createTimer(ros::Duration(1),
			&EngineControl::publishTimerCallback, this);
	publishTimer.setPeriod(ros::Duration(1.0 / publishFrequency));

	// Registrierung der Service Clients.
	depthPid  = node.serviceClient<hanse_pidcontrol::Enable>("/hanse/pid/depth/enable");
	orientationPid  = node.serviceClient<hanse_pidcontrol::Enable>("/hanse/pid/orientation/enable");
	rotationSpeedPid  = node.serviceClient<hanse_pidcontrol::Enable>("/hanse/pid/rotationSpeed/enable");

	hanse_pidcontrol::Enable enableMsg;
	enableMsg.request.enable = true;

	// Aktivierung der Standard-PID-Regler.
	ros::Rate loopRate(10);
 	while(ros::ok()) {
		if (depthPid.call(enableMsg) && orientationPid.call(enableMsg)) {
			ROS_INFO("PIDs enabled.");
			break;
		}
		else {
			ROS_INFO("PIDs could not be enabled.");	
		}

		loopRate.sleep();
	}

	ROS_INFO("engine_control started");
}

// Auswertung und Zwischenspeicherung der Eingabedaten des Drucksensors.
void EngineControl::pressureCallback(
		const hanse_msgs::pressure::ConstPtr& pressure) {

	boost::mutex::scoped_lock(mutex);

	if (!pressureInit) {
		pressureBias = pressure->data;
		pressureInit = true;
	}

	pressureCurrent = pressure->data;
}

// Auswertung und Zwischenspeicherung der cmd_vel Nachricht.
void EngineControl::velocityCallback(
		const geometry_msgs::Twist::ConstPtr& twist) {

	boost::mutex::scoped_lock(mutex);

	depthTarget = twist->linear.z;
	linearSpeedTarget = twist->linear.x;
	rotationSpeedTarget = twist->angular.z;
}

// Auswertung und Zwischenspeicherung der XSens Eingabedaten.
void EngineControl::xsensCallback(
		const sensor_msgs::Imu::ConstPtr& xsensData) {

	boost::mutex::scoped_lock(mutex);

	// Eingabe bei aktivierter Handcontrol speichern.
	rotationSpeedCurrent = xsensData->angular_velocity.z * rotationSpeedScale * orientationZ;

	// Konvertierung der Quaternion zu RPY bei deaktivierter Handcontrol.
	btQuaternion q;
	double pitch, yaw;
	tf::quaternionMsgToTF(xsensData->orientation, q);
	btMatrix3x3(q).getRPY(yaw, pitch, orientationCurrent);
	orientationCurrent = orientationCurrent * orientationZ * (-1);
}

// Wechseln zwischen den PID-Reglern für die Drehbewegung. Direkter Zusammenhang
// mit der Handcontrol.
void EngineControl::switchPids(
		const std_msgs::Bool::ConstPtr& enableRotationSpeedPid) {

	boost::mutex::scoped_lock(mutex);

	// Nachrichten anlegen.
	hanse_pidcontrol::Enable rotationSpeedPidMsg;
	rotationSpeedPidMsg.request.enable = enableRotationSpeedPid->data;
	hanse_pidcontrol::Enable orientationPidMsg;
	orientationPidMsg.request.enable = !(enableRotationSpeedPid->data);

	ros::Rate loopRate(10);

	while(ros::ok()) {
		if (rotationSpeedPid.call(rotationSpeedPidMsg) && orientationPid.call(orientationPidMsg)) {
			ROS_INFO("RotationSpeed-PID enabled, Orientation-PID disabled.");
			break;
		} else {
			ROS_INFO("PIDs could not be switched.");	
		}

		loopRate.sleep();
	}

    if (enableRotationSpeedPid->data) {
        orientationOutput = 0;
    } else {
        rotationSpeedOutput = 0;
    }
}

// Auswertung und Zwischenspeicherung der Ausgabedaten des Druck-PID-Reglers.
void EngineControl::depthOutputCallback(
		const std_msgs::Float64::ConstPtr& depthOutput) {

	boost::mutex::scoped_lock(mutex);

	this->depthOutput = depthOutput->data;
}

// Auswertung und Zwischenspeicherung der Orientierung.
void EngineControl::orientationTargetCallback(
		const std_msgs::Float64::ConstPtr& orientationTarget) {

	boost::mutex::scoped_lock(mutex);

	this->orientationTarget = orientationTarget->data;
}

// Auswertung und Zwischenspeicherung der Ausgabedaten des Orientierungs-PID-Reglers.
void EngineControl::orientationOutputCallback(
		const std_msgs::Float64::ConstPtr& orientationOutput) {

	boost::mutex::scoped_lock(mutex);

	this->orientationOutput = orientationOutput->data;
}

// Auswertung und Zwischenspeicherung der Ausgabedaten des Drehgeschwindigkeits-PID-Reglers.
void EngineControl::rotationSpeedOutputCallback(
		const std_msgs::Float64::ConstPtr& rotationSpeedOutput) {

	boost::mutex::scoped_lock(mutex);

	this->rotationSpeedOutput = rotationSpeedOutput->data;
}

// Ausführung jeweils nach Ablauf des Timers. Wird verwendet, um sämtliche
// Ausgaben auf die vorgesehenen Topics zu schreiben.
void EngineControl::publishTimerCallback(const ros::TimerEvent &e) {

	boost::mutex::scoped_lock(mutex);

	// Tiefensteuerung.
	std_msgs::Float64 depthTargetMsg;
	depthTargetMsg.data = depthTarget + pressureBias;

	pubDepthTarget.publish(depthTargetMsg);

	std_msgs::Float64 depthCurrentMsg;
	depthCurrentMsg.data = pressureCurrent;

	pubDepthCurrent.publish(depthCurrentMsg);

	// Drehgeschwindigkeitssteuerung.
	std_msgs::Float64 rotationSpeedTargetMsg;
	rotationSpeedTargetMsg.data = rotationSpeedTarget;

	std_msgs::Float64 rotationSpeedCurrentMsg;
	rotationSpeedCurrentMsg.data = rotationSpeedCurrent;

	pubRotationSpeedTarget.publish(rotationSpeedTargetMsg);
	pubRotationSpeedCurrent.publish(rotationSpeedCurrentMsg);

	// Orientierungssteuerung.
	std_msgs::Float64 orientationCurrentMsg;
	double rotation;
	double absoluteDistance = std::abs(orientationTarget) + std::abs(orientationCurrent);

	if(absoluteDistance < M_PI || absoluteDistance == std::abs(orientationTarget + orientationCurrent)) {
		rotation = -(orientationTarget - orientationCurrent);
	} else {
		absoluteDistance = orientationTarget - orientationCurrent;

		if(absoluteDistance < 0.0) {
			rotation = -(absoluteDistance + 2 * M_PI);
		} else {
			rotation = -(absoluteDistance - 2 * M_PI);
		}
	}

	orientationCurrentMsg.data = rotation;
	pubOrientationCurrent.publish(orientationCurrentMsg);


	// Motorsteuerung
	int16_t motorLeft = 0;
	int16_t motorRight = 0;

	// Berechnung der Ansteuerungsstärke der seitlichen Motoren.
	motorLeft = linearSpeedTarget * 127 - rotationSpeedOutput - orientationOutput;
	motorRight = linearSpeedTarget * 127 + rotationSpeedOutput + orientationOutput;
	
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

	hanse_msgs::sollSpeed motorLeftMsg;
	motorLeftMsg.data = motorLeft;

	hanse_msgs::sollSpeed motorRightMsg;
	motorRightMsg.data = motorRight;

	pubMotorLeft.publish(motorLeftMsg);
	pubMotorRight.publish(motorRightMsg);

	hanse_msgs::sollSpeed motorHeightMsg;
	motorHeightMsg.data = -depthOutput;

	pubMotorFront.publish(motorHeightMsg);
	pubMotorRear.publish(motorHeightMsg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "engine_control");

	ros::start();

	EngineControl engine_control;

	ros::spin();
}
