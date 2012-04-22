#include <cfloat>
#include "angles/angles.h"
#include "sensor_msgs/LaserScan.h"
#include "online_wall_detection.h"

OnlineWallDetection::OnlineWallDetection(ros::NodeHandle handle) :
    nh(handle),
    publisher(handle.advertise<sensor_msgs::LaserScan>("sonar/laser_scan", 1)),
    debugPublisher(handle.advertise<hanse_msgs::ScanningSonar>("sonar/scan/wall_detection", 1)),
    subscriber(handle.subscribe("sonar/scan", 1, &OnlineWallDetection::callback, this))
{
    reconfigServer.setCallback(boost::bind(&OnlineWallDetection::reconfigure, this, _1, _2));
}

void OnlineWallDetection::callback(const hanse_msgs::ScanningSonar &msg)
{
    lastMessages.push_front(msg);

    Eigen::ArrayXf echoData(msg.echoData.size());
    for (unsigned i = 0; i < msg.echoData.size(); i++) {
	echoData(i) = msg.echoData[i] / 255.0;
    }

    Eigen::ArrayXf ones;
    ones.setOnes(echoData.rows());

    echoData = (nmsFilter(wallFilter(minFilter(echoData, config.min_filter_width)), config.nms_filter_width) * config.pre_gain).min(ones);
    for (unsigned i = config.length_cutoff; i < msg.echoData.size(); i++) {
	echoData[i] = 0;
    }

    echoDatas.push_front(echoData);

    if (echoDatas.size() > 3) {
	echoDatas.pop_back();
	lastMessages.pop_back();
    }

    if (echoDatas.size() == 3) {
	const Eigen::ArrayXf &left = echoDatas[0];
	const Eigen::ArrayXf &mid = echoDatas[1];
	const Eigen::ArrayXf &right = echoDatas[2];

	if (!(left.rows() == mid.rows() && mid.rows() == right.rows()))
	    return;

	int dataPoints = mid.rows();

	const auto &midMsg = lastMessages[1];


	Eigen::ArrayXf leftInnerSlope = slopeFilter(left, 0, config.between_slope);
	Eigen::ArrayXf rightInnerSlope = slopeFilter(right, 0, config.between_slope);
	Eigen::ArrayXf leftOuterSlope = slopeFilter(left, config.between_slope, 0);
	Eigen::ArrayXf rightOuterSlope = slopeFilter(right, config.between_slope, 0);
	Eigen::ArrayXf leftOutside = slopeFilter(left, config.outside_slope);
	Eigen::ArrayXf rightOutside = slopeFilter(right, config.outside_slope);
	Eigen::ArrayXf betweenA = leftInnerSlope.min(rightOuterSlope);
	Eigen::ArrayXf betweenB = rightInnerSlope.min(leftOuterSlope);
	Eigen::ArrayXf between = betweenA.max(betweenB).max(leftOutside).max(rightOutside);

	Eigen::ArrayXf midFiltered = mid * between;

	for (int i = 0; i < dataPoints; i++) {
	    if (midFiltered(i) > config.threshold) {
		midFiltered(i) = 1.0;
	    } else {
		midFiltered(i) = 0.0;
	    }
	}

	debugPublisher.publish(debugMessage(midFiltered, 1));

	double distance = 0.0;

	for (int i = 0; i < dataPoints; i++) {
	    if (midFiltered(i) > 0.5) {
		distance = ((double)i / (dataPoints-1)) * (midMsg.range - 0.15) + 0.15;
		break;
	    }
	}


	double headPosition = angles::normalize_angle(msg.headPosition);

	sensor_msgs::LaserScan laserScan;
	laserScan.header.stamp = midMsg.header.stamp;
	laserScan.header.frame_id = "/map";
	laserScan.angle_min = headPosition;
	laserScan.angle_max = headPosition;
	laserScan.angle_increment = 0;
	laserScan.time_increment = 0;
	laserScan.scan_time = 0;
	laserScan.range_min = 0.15;
	laserScan.range_max = msg.range;
	laserScan.ranges.push_back(distance);
	publisher.publish(laserScan);

    }
}

Eigen::ArrayXf OnlineWallDetection::minFilter(Eigen::ArrayXf const &data, unsigned width)
{
    Eigen::ArrayXf minfilter(data.rows());

    for (unsigned i = 0; i < data.rows(); i++) {
	float f = 1.0;
	for (int k = -width; k <= (int)width; k++) {
	    int j = i + k;
	    if (j < 0 || j >= data.rows())
		continue;
	    f = std::min(f, data(j) + (float)(abs(k))/((width+1)));
	}
	minfilter(i) = f;
    }
    return minfilter;
}

Eigen::ArrayXf OnlineWallDetection::integralFilter(Eigen::ArrayXf const &data)
{
    Eigen::ArrayXf integral(data.rows());
    float sum = 0;
    for (unsigned i = 0; i < data.rows(); i++) {
	sum += data(i);
	integral(i) = sum;
    }
    return integral;
}


Eigen::ArrayXf OnlineWallDetection::densityFilter(Eigen::ArrayXf const &data)
{
    Eigen::ArrayXf linear(data.rows());
    Eigen::ArrayXf integral = integralFilter(data);
    Eigen::ArrayXf integralNorm = integral / data.sum();

    for (unsigned i = 0; i < data.rows(); i++) {
	linear(i) = (float)i / (data.rows()-1);
    }
    return integralNorm/(linear+0.01) - (1 - integralNorm)/(1.01-linear);
}


Eigen::ArrayXf OnlineWallDetection::shiftFilter(Eigen::ArrayXf const &data, int amount)
{
    Eigen::ArrayXf shifted(data.rows());
    for (unsigned i = 0; i < data.rows(); i++) {
	int k = amount + (int)i;
	if (k < 0)
	    shifted(i) = data(0);
	else if (k >= data.rows())
	    shifted(i) = data(data.rows()-1);
	else
	    shifted(i) = data(k);
    }
    return shifted;
}

Eigen::ArrayXf OnlineWallDetection::wallFilter(Eigen::ArrayXf const &data)
{
    Eigen::ArrayXf result;
    result.setOnes(data.rows());
    Eigen::ArrayXf integral = integralFilter(data);
    Eigen::ArrayXf zeros;
    zeros.setZero(data.rows());
    for (unsigned c = 0; c < 6; c++) {
	int w = 1 << c;
	result *= (2 * integral - shiftFilter(integral, -w) - shiftFilter(integral, w)).max(zeros);
    }
    return result;
}

Eigen::ArrayXf OnlineWallDetection::slopeFilter(Eigen::ArrayXf const &data, float inner, float outer)
{
    Eigen::ArrayXf temp(data.rows()), result(data.rows());
    float val = -FLT_MAX;
    for (unsigned i = 0; i < data.rows(); i++) {
	val -= 1/inner;
	val = std::max(val, data(data.rows() - 1 - i));
	temp(i) = val;
    }
    val = -FLT_MAX;
    for (unsigned i = 0; i < data.rows(); i++) {
	val -= 1/outer;
	val = std::max(val, temp(data.rows() - 1 - i));
	result(i) = val;
    }
    return result;
}

Eigen::ArrayXf OnlineWallDetection::nmsFilter(Eigen::ArrayXf const &data, unsigned width)
{
    Eigen::ArrayXf result(data.rows());

    for (unsigned i = 0; i < data.rows(); i++) {
	float m = 0.0;
	for (int k = -width; k <= (int)width; k++) {
	    int j = i + k;
	    if (j < 0 || j >= data.rows())
		continue;
	    m = std::max(m, data(j));
	}
	if (m == data(i))
	    result(i) = m;
	else
	    result(i) = 0;
    }
    return result;
}



hanse_msgs::ScanningSonar OnlineWallDetection::debugMessage(Eigen::ArrayXf const &data, unsigned i)
{
    hanse_msgs::ScanningSonar msg(lastMessages[i]);
    msg.echoData.clear();
    for (unsigned i = 0; i < data.rows(); i++) {
	msg.echoData.push_back(std::max(0.0f,std::min(1.0f,data(i))) * 255.0);
    }
    return msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sonar_online_wall_detection");
    ros::NodeHandle n;

    OnlineWallDetection wallDetection(n);

    ros::spin();

    return 0;
}
