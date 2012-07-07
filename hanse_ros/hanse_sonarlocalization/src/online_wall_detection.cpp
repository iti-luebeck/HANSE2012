#include <cfloat>
#include "angles/angles.h"
#include "hanse_msgs/WallDetection.h"
#include "online_wall_detection.h"

OnlineWallDetection::OnlineWallDetection(ros::NodeHandle handle) :
    nh(handle),
    publisher(handle.advertise<hanse_msgs::WallDetection>("sonar/scan/walls", 1)),
    debugPublisher(handle.advertise<hanse_msgs::ScanningSonar>("sonar/scan/wall_detection", 1)),
    debugPublisher2(handle.advertise<hanse_msgs::ScanningSonar>("sonar/scan/wall_detection/debug", 1)),
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
    Eigen::ArrayXf zeros;
    zeros.setZero(echoData.rows());

    Eigen::ArrayXf slope(echoData.rows());

    for (unsigned i = 0; i < echoData.rows(); i++)
        slope(i) = (float)i / echoData.rows();

    //echoData = densityFilter(echoData);
    Eigen::ArrayXf damping = config.distance_damping * slope;;
    echoData = (echoData - damping).max(zeros) * (ones / (1 - damping));
    //echoData = (echoData - medianFilter(echoData, config.min_filter_width, 0.5)).max(zeros) * 4;
    echoData = medianFilter(echoData, config.median_filter_width, config.median_filter_mu);
    echoData = wallFilter(echoData);
    echoData = (nmsFilter(echoData, config.nms_filter_width) * config.pre_gain).min(ones);
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

        debugPublisher2.publish(debugMessage(between, 0));


	Eigen::ArrayXf midFiltered = mid * between;

	hanse_msgs::WallDetection wall;

	for (int i = 0; i < dataPoints; i++) {
	    if (midFiltered(i) > config.threshold) {
		wall.distances.push_back(((double)i / (dataPoints-1)) * (midMsg.range - 0.15) + 0.15);
		midFiltered(i) = 1.0;
	    } else {
		midFiltered(i) = 0.0;
	    }
	}

	debugPublisher.publish(debugMessage(midFiltered, 1));


	double headPosition = angles::normalize_angle(msg.headPosition);

	wall.header.stamp = midMsg.header.stamp;
	wall.header.frame_id = "/map";
	// the angles from sonar are cw/negative when viewing the map
	// from the top, we only work with ccw/positive angles
	wall.headPosition = M_PI / 2 - headPosition;
	wall.range = midMsg.range;
	wall.wallDetected = !wall.distances.empty();
	publisher.publish(wall);

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

Eigen::ArrayXf OnlineWallDetection::medianFilter(Eigen::ArrayXf const &data, unsigned width, float mu)
{
    Eigen::ArrayXf medianfilter(data.rows());

    float buffer[2 * width + 1];

    for (unsigned i = 0; i < data.rows(); i++) {
	for (int k = -width; k <= (int)width; k++) {
	    int j = i + k;
            float val;
	    if (j < 0)
                val = 0;
            else if (j >= data.rows())
                val = 0;
            else
                val = data(j);
            buffer[k+width] = val;
	}
        std::sort(buffer, buffer + 2 * width + 1);
	medianfilter(i) = buffer[(int)(mu * 2 * width)];
    }
    return medianfilter;
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
