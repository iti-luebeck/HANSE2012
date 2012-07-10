#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>

#include "hanse_srvs/GetOutput.h"
#include "hanse_srvs/SetTarget.h"
#include "hanse_srvs/Bool.h"

#include "hanse_pidcontrol/PidcontrolConfig.h"

const std::string input_topic_ = "input";
const std::string target_topic_ = "target";
const std::string output_topic_ = "output";

class PIDControl
{
private:

	// **** ros-related variables

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Publisher output_publisher_;
	ros::Subscriber input_subscriber_;
	ros::Subscriber target_subscriber_;

	ros::ServiceServer enable_srv_;
	ros::ServiceServer get_output_srv_;

    ros::Timer publish_timer_;

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

    bool enable(hanse_srvs::Bool::Request &req,
            hanse_srvs::Bool::Response &res);
    bool getOutput(hanse_srvs::GetOutput::Request &req,
            hanse_srvs::GetOutput::Response &res);
    bool setTarget(hanse_srvs::SetTarget::Request &req,
            hanse_srvs::SetTarget::Response &res);

	// **** member functions

	void initParams();
	void inputCallback(const std_msgs::Float64Ptr& input_msg);
	void targetCallback(const std_msgs::Float64Ptr& target_msg);
	void publishCallback(const ros::TimerEvent& event);

    // **** dynamic reconfigure

    void dynReconfigureCallback(hanse_pidcontrol::PidcontrolConfig &config_, uint32_t level);
    hanse_pidcontrol::PidcontrolConfig config_;

    /** \brief dynamic_reconfigure interface */
    dynamic_reconfigure::Server<hanse_pidcontrol::PidcontrolConfig> dyn_reconfigure_srv_;

    /** \brief dynamic_reconfigure call back */
    dynamic_reconfigure::Server<hanse_pidcontrol::PidcontrolConfig>::CallbackType dyn_reconfigure_cb_;

public:

	PIDControl();
	virtual ~PIDControl();
};

PIDControl::PIDControl()
{
	ROS_INFO("Starting PIDControl");

	output_msg_ = boost::make_shared<std_msgs::Float64>();

	target_ = 0.0;
	i_ = 0.0;
	received_input_ = false;
	enabled_ = false;

    dyn_reconfigure_cb_ = boost::bind(&PIDControl::dynReconfigureCallback, this, _1, _2);
    dyn_reconfigure_srv_.setCallback(dyn_reconfigure_cb_);

    // initParams();

	// **** subscribers

	input_subscriber_ = nh_.subscribe(input_topic_, 1,
			&PIDControl::inputCallback, this);
	target_subscriber_ = nh_.subscribe(target_topic_, 1,
			&PIDControl::targetCallback, this);

	// **** publishers

	output_publisher_ = nh_.advertise<std_msgs::Float64>(output_topic_, 1);
	// TODO: Do we need to publish in a loop here? or would 1 time suffice
    publish_timer_ = nh_.createTimer(ros::Duration(1),
			&PIDControl::publishCallback, this);

	// **** services

	enable_srv_ = nh_.advertiseService("enable", &PIDControl::enable, this);
	get_output_srv_ = nh_.advertiseService("getOutput", &PIDControl::getOutput,
			this);
}

PIDControl::~PIDControl()
{
	std::cout << "Destroying PIDControl." << std::endl;
}

void PIDControl::dynReconfigureCallback(hanse_pidcontrol::PidcontrolConfig &config, uint32_t level)
{
    ROS_INFO("got new parameters, level=%d", level);

    config_ = config;

    target_ = 0.0;
    i_ = 0.0;
    received_input_ = false;
    prev_error_ = 0.0;

    publish_timer_.setPeriod(ros::Duration(1.0/config.publish_frequency));
}

void PIDControl::targetCallback(const std_msgs::Float64Ptr& target_msg)
{
	boost::mutex::scoped_lock(mutex_);
	target_ = target_msg->data;

	ROS_DEBUG("PID Target: %f", target_);
}

void PIDControl::inputCallback(const std_msgs::Float64Ptr& input_msg)
{
	boost::mutex::scoped_lock(mutex_);
	if (!enabled_)
    {
		return;
    }

	double input = input_msg->data;

	// TODO: new Topic with time
	ros::Time input_stamp = ros::Time::now();

    // ROS_INFO("PID: Input: %f \t Target: %f", input, target_);

	// **** calculate error

	double error = target_ - input;

    if (config_.angular) // wrap the error to (-pi, pi]
	{
		while (error > M_PI)
			error -= M_PI * 2.0;
		while (error <= M_PI)
			error += M_PI * 2.0;
	}

	// **** calculate P, I, D terms

	double p_term, i_term, d_term;
    p_term = config_.k_p * error;

    if (received_input_)
    {
		double dt = (input_stamp - prev_input_stamp_).toSec();

		i_ += error * dt;
        if (i_ > config_.i_max)
        {
            //ROS_WARN("PID Control integral wind-up: clamping to %f", config.i_max);
            i_ = config_.i_max;
		}

        if (i_ < -config_.i_max)
        {
            //ROS_WARN("PID Control integral wind-down: clamping to %f", -config.i_max);
            i_ = -config_.i_max;
		}

        i_term = config_.k_i * i_;

        d_term = config_.k_d * (error - prev_error_) / dt;
    }
    else
    {
		received_input_ = true;
		i_term = 0.0;
		d_term = 0.0;
	}

	// **** calculate output

    double output = p_term + i_term + d_term + config_.bias;
    if (output < config_.output_min)
    {
        output = config_.output_min; // clamp to min value
	}

    if (output > config_.output_max)
    {
        output = config_.output_max; // clamp to max value
	}

	output_msg_->data = output;
	output_publisher_.publish(output_msg_);

	// **** rotate variables

	prev_input_stamp_ = input_stamp;
	prev_error_ = error;
}

void PIDControl::publishCallback(const ros::TimerEvent& event)
{
	boost::mutex::scoped_lock(mutex_);
	if (!enabled_)
    {
		return;
    }

	output_publisher_.publish(output_msg_);
}

bool PIDControl::enable(hanse_srvs::Bool::Request &req,
        hanse_srvs::Bool::Response &res)
{
	boost::mutex::scoped_lock(mutex_);
	if (req.enable) {
		enabled_ = true;
	} else {
        target_ = 0.0;
        i_ = 0.0;
        received_input_ = false;
        prev_error_ = 0.0;
        enabled_ = false;
	}

	return true;
}

bool PIDControl::getOutput(hanse_srvs::GetOutput::Request &req,
        hanse_srvs::GetOutput::Response &res)
{
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
