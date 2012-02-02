#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <hanse_msgs/pressure.h>
#include <hanse_msgs/sollSpeed.h>
#include <hanse_pidcontrol/GetOutput.h>

class EngineControl {
public:
	EngineControl();

private:
	ros::NodeHandle node;

	ros::Subscriber sub_velocity;
	ros::Subscriber sub_pressure;
	ros::Subscriber sub_pid_depth_output;

	ros::Publisher pub_depth_target;
	ros::Publisher pub_depth_input;
	ros::Publisher pub_depth_output_front;
	ros::Publisher pub_depth_output_rear;
	ros::Publisher pub_linear_output_left;
	ros::Publisher pub_linear_output_right;

	double value_linear;
	double value_depth;
	double value_angular;

	uint16_t pressure_zero;
	uint16_t pressure_curr;
	bool pressure_init;
	bool target_changed;

	ros::Timer control_loop_timer;

	void velocityCallback(const geometry_msgs::Twist::ConstPtr& twist);
	void pressureCallback(const hanse_msgs::pressure::ConstPtr& press);
	void depthOutputCallback(const std_msgs::Float64::ConstPtr& depthOutput);
	void timerCallback(const ros::TimerEvent &e);
};

EngineControl::EngineControl() :
		value_linear(0), value_depth(0), value_angular(0), pressure_zero(0), pressure_curr(
				0), pressure_init(false), target_changed(false) {

	pub_depth_target = node.advertise<std_msgs::Float64>("/depth_pid/target",
			1);
	pub_depth_input = node.advertise<std_msgs::Float64>("/depth_pid/input", 1);
	pub_depth_output_front = node.advertise<hanse_msgs::sollSpeed>(
			"/hanse2/thrusterDownFront", 1);
	pub_depth_output_rear = node.advertise<hanse_msgs::sollSpeed>(
			"/hanse2/thrusterDown", 1);
	pub_linear_output_left = node.advertise<hanse_msgs::sollSpeed>(
			"/hanse2/thrusterLeft", 1);
	pub_linear_output_right = node.advertise<hanse_msgs::sollSpeed>(
			"/hanse2/thrusterRight", 1);
	sub_velocity = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
			&EngineControl::velocityCallback, this);
	sub_pressure = node.subscribe<hanse_msgs::pressure>("/hanse2/press", 10,
			&EngineControl::pressureCallback, this);
	sub_pid_depth_output = node.subscribe<std_msgs::Float64>(
			"/depth_pid/output", 10, &EngineControl::depthOutputCallback, this);

	// will be set to actual value once config is loaded
	control_loop_timer = node.createTimer(ros::Duration(1),
			&EngineControl::timerCallback, this);
	control_loop_timer.setPeriod(ros::Duration(1.0 / 10));

	ROS_INFO("engine_control started");
}

void EngineControl::velocityCallback(
		const geometry_msgs::Twist::ConstPtr& twist) {

	int16_t left = 0;
	int16_t right = 0;

	if (value_depth != twist->linear.z) {
		value_depth = twist->linear.z;
		target_changed = true;
	} else {
		target_changed = false;
	}

	value_linear = twist->linear.x;
	value_angular = twist->angular.z;

	left = value_linear*127 + value_angular*127;
	right = value_linear*127 - value_angular*127;

	if(left > 127) {
		left = 127;
	} else if(left < -127) {
		left = -127;
	}

	if(right > 127) {
		right = 127;
	} else if(right < -127) {
		right = -127;
	}

	hanse_msgs::sollSpeed thrusterLeft;
	thrusterLeft.data = left;

	hanse_msgs::sollSpeed thrusterRight;
	thrusterRight.data = right;

	pub_linear_output_left.publish(thrusterLeft);
	pub_linear_output_right.publish(thrusterRight);
}

void EngineControl::pressureCallback(
		const hanse_msgs::pressure::ConstPtr& press) {
	if (!pressure_init) {
		pressure_zero = press->data;
		pressure_init = true;
	}

	pressure_curr = press->data;
}

void EngineControl::depthOutputCallback(
		const std_msgs::Float64::ConstPtr& depthOutput) {
	int8_t outputInt = (int8_t) depthOutput->data;

	hanse_msgs::sollSpeed thrusterDown;
	thrusterDown.data = -outputInt;

	pub_depth_output_front.publish(thrusterDown);
	pub_depth_output_rear.publish(thrusterDown);
	
	std::cout << "Thruster " << -outputInt << std::endl;
}

void EngineControl::timerCallback(const ros::TimerEvent &e) {
	// publish data here.
	// DEPTH CONTROL
	std_msgs::Float64 targetDepth;
	targetDepth.data = pressure_zero + value_depth;

	pub_depth_target.publish(targetDepth);

	if (target_changed) {
		std::cout << "Forward " << value_linear << " Angular " << value_angular
				<< " Depth " << value_depth << std::endl;
	}

	std_msgs::Float64 input;
	input.data = pressure_curr;

	pub_depth_input.publish(input);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "engine_control");

	ros::start();

	EngineControl engine_control;

	ros::spin();
}
