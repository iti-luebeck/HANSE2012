#ifndef ONLINE_WALL_DETECTION_H
#define ONLINE_WALL_DETECTION_H

#include <deque>
#include <Eigen/Core>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "hanse_msgs/ScanningSonar.h"
#include "hanse_sonarlocalization/OnlineWallDetectionConfig.h"
#include "dynamic_reconfigure/server.h"

class OnlineWallDetection
{
public:
    OnlineWallDetection(ros::NodeHandle handle);

private:
    class WallDistance
    {
    public:
	double headPosition;
	double distance;
    };

    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Publisher debugPublisher;
    ros::Subscriber subscriber;

    std::deque<Eigen::ArrayXf> echoDatas;
    std::deque<hanse_msgs::ScanningSonar> lastMessages;

    dynamic_reconfigure::Server<hanse_sonarlocalization::OnlineWallDetectionConfig> reconfigServer;
    hanse_sonarlocalization::OnlineWallDetectionConfig config;

    void reconfigure(hanse_sonarlocalization::OnlineWallDetectionConfig &newConfig, uint32_t level) { config = newConfig; }
    void callback(const hanse_msgs::ScanningSonar &msg);

    Eigen::ArrayXf minFilter(Eigen::ArrayXf const &data, unsigned width);
    Eigen::ArrayXf integralFilter(Eigen::ArrayXf const &data);
    Eigen::ArrayXf densityFilter(Eigen::ArrayXf const &data);
    Eigen::ArrayXf shiftFilter(Eigen::ArrayXf const &data, int amount);
    Eigen::ArrayXf wallFilter(Eigen::ArrayXf const &data);
    Eigen::ArrayXf slopeFilter(Eigen::ArrayXf const &data, float slope) { return slopeFilter(data, slope, slope); }
    Eigen::ArrayXf slopeFilter(Eigen::ArrayXf const &data, float inner, float outer);
    Eigen::ArrayXf nmsFilter(Eigen::ArrayXf const &data, unsigned width);

    hanse_msgs::ScanningSonar debugMessage(unsigned i = 0) { return debugMessage(echoDatas[i], i); }
    hanse_msgs::ScanningSonar debugMessage(Eigen::ArrayXf const &data, unsigned i = 0);
};

#endif
