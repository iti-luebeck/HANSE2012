#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/thread/mutex.hpp>
#include "hanse_pidcontrol/GetOutput.h"
#include "hanse_pidcontrol/SetTarget.h"
#include "hanse_pidcontrol/Enable.h"

const std::string input_topic_ = "input";
const std::string target_topic_ = "target";
const std::string output_topic_ = "output";

class PIDControl {
private:

	// **** ros-related variables

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Publisher output_publisher_;
	ros::Subscriber input_subscriber_;
	ros::Subscriber target_subscriber_;

	ros::ServiceServer enable_srv_;
	ros::ServiceServer get_output_srv_;

	// **** parameters

	double frequency_;
	double k_p_;
	double k_i_;
	double k_d_;
	double bias_;
	double i_max_;
	double output_min_;
	double output_max_;
	bool angular_; // if true, then wraps the error to (-pi, pi]

	// **** state variables

	bool enabled_;

	boost::mutex mutex_;

	std_msgs::Float64Ptr output_msg_;

	double i_;
	double target_;

	bool received_input_;
	ros::Time prev_input_stamp_;
	double prev_error_;

	// **** services

	bool enable(hanse_pidcontrol::Enable::Request &req,
			hanse_pidcontrol::Enable::Response &res);
	bool getOutput(hanse_pidcontrol::GetOutput::Request &req,
			hanse_pidcontrol::GetOutput::Response &res);
	bool setTarget(hanse_pidcontrol::SetTarget::Request &req,
			hanse_pidcontrol::SetTarget::Response &res);

	// **** member functions

	void initParams();
	void inputCallback(const std_msgs::Float64Ptr& input_msg);
	void targetCallback(const std_msgs::Float64Ptr& target_msg);
	void publishCallback(const ros::TimerEvent& event);

public:

	PIDControl();
	virtual ~PIDControl();
};

PIDControl::PIDControl() {
	ROS_INFO("Starting PIDControl");

	output_msg_ = boost::make_shared<std_msgs::Float64>();

	target_ = 0.0;
	i_ = 0.0;
	received_input_ = false;
	enabled_ = false;

	initParams();

	// **** subscribers

	input_subscriber_ = nh_.subscribe(input_topic_, 1,
			&PIDControl::inputCallback, this);
	target_subscriber_ = nh_.subscribe(target_topic_, 1,
			&PIDControl::targetCallback, this);

	// **** publishers

	output_publisher_ = nh_.advertise<std_msgs::Float64>(output_topic_, 1);
	// TODO: Do we need to publish in a loop here? or would 1 time suffice
	ros::Timer timer = nh_.createTimer(ros::Duration(1.0 / frequency_),
			&PIDControl::publishCallback, this);

	// **** services

	enable_srv_ = nh_.advertiseService("enable", &PIDControl::enable, this);
	get_output_srv_ = nh_.advertiseService("getOutput", &PIDControl::getOutput,
			this);
}

PIDControl::~PIDControl() {
	std::cout << "Destroying PIDControl." << std::endl;
}

void PIDControl::initParams() {
	if (!nh_private_.getParam("k_p", k_p_))
		ROS_FATAL("k_p needs to be set");
	if (!nh_private_.getParam("k_i", k_i_))
		ROS_ERROR("k_i needs to be set");
	if (!nh_private_.getParam("k_d", k_d_))
		ROS_FATAL("k_d needs to be set");
	if (!nh_private_.getParam("bias", bias_))
		bias_ = 0.0;
	if (!nh_private_.getParam("i_max", i_max_))
		ROS_FATAL("i_max needs to be set");
	if (!nh_private_.getParam("frequency", frequency_))
		frequency_ = 10.0;
	if (!nh_private_.getParam("output_min", output_min_))
		ROS_FATAL("output_min needs to be set");
	if (!nh_private_.getParam("output_max", output_max_))
		ROS_FATAL("output_max needs to be set");
	if (!nh_private_.getParam("angular", angular_))
		angular_ = false;
}

void PIDControl::targetCallback(const std_msgs::Float64Ptr& target_msg) {
	boost::mutex::scoped_lock(mutex_);
	target_ = target_msg->data;

	ROS_DEBUG("PID Target: %f", target_);
}

void PIDControl::inputCallback(const std_msgs::Float64Ptr& input_msg) {
	boost::mutex::scoped_lock(mutex_);
	if (!enabled_)
		return;

	double input = input_msg->data;

	// TODO: new Topic with time
	ros::Time input_stamp = ros::Time::now();

	ROS_INFO("PID: Input: %f \t Target: %f", input, target_);

	// **** calculate error

	double error = target_ - input;

	if (angular_) // wrap the error to (-pi, pi]
	{
		while (error > M_PI)
			error -= M_PI * 2.0;
		while (error <= M_PI)
			error += M_PI * 2.0;
	}

	// **** calculate P, I, D terms

	double p_term, i_term, d_term;
	p_term = k_p_ * error;

	if (received_input_) {
		double dt = (input_stamp - prev_input_stamp_).toSec();

		i_ += error * dt;
		if (i_ > i_max_) {
			ROS_WARN("PID Control integral wind-up: clamping to %f", i_max_);
			i_ = i_max_;
		}
		if (i_ < -i_max_) {
			ROS_WARN("PID Control integral wind-down: clamping to %f", -i_max_);
			i_ = -i_max_;
		}

		i_term = k_i_ * i_;

		d_term = k_d_ * (error - prev_error_) / dt;
	} else {
		received_input_ = true;
		i_term = 0.0;
		d_term = 0.0;
	}

	// **** calculate output

	double output = p_term + i_term + d_term + bias_;
	if (output < output_min_) {
		ROS_WARN(
				"PID Control output too small, clamping to to %f", output_min_);
		output = output_min_; // clamp to min value
	}
	if (output > output_max_) {
		ROS_WARN("PID Control output too big, clamping to to %f", output_max_);
		output = output_max_; // clamp to max value
	}

	//ROS_INFO("PID: Output: %f (%f + %f + %f + %f)", output, p_term, i_term, d_term, bias_);

	output_msg_->data = output;
	output_publisher_.publish(output_msg_);

	// **** rotate variables

	prev_input_stamp_ = input_stamp;
	prev_error_ = error;
}

void PIDControl::publishCallback(const ros::TimerEvent& event) {
	boost::mutex::scoped_lock(mutex_);
	if (!enabled_)
		return;

	ROS_INFO("Publishing %f", output_msg_->data);

	output_publisher_.publish(output_msg_);

}

bool PIDControl::enable(hanse_pidcontrol::Enable::Request &req,
		hanse_pidcontrol::Enable::Response &res) {
	ROS_INFO("Service call: hanse_pidcontrol");

	boost::mutex::scoped_lock(mutex_);
	if (req.enable) {
		ROS_INFO("Enabling hanse_pidcontrol");
		enabled_ = true;
	} else {
		ROS_INFO("Disabling hanse_pidcontrol");
		enabled_ = false;
	}

	ROS_INFO("Service leave: hanse_pidcontrol");
	return true;
}

bool PIDControl::getOutput(hanse_pidcontrol::GetOutput::Request &req,
		hanse_pidcontrol::GetOutput::Response &res) {
	boost::mutex::scoped_lock(mutex_);
	res.output = output_msg_->data;
	return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PIDControl");

  ros::start();

  PIDControl pid_control;

  ros::spin();

  return 0;
}
