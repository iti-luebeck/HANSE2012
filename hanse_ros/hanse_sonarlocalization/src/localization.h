#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <deque>
#include <Eigen/Geometry>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "dynamic_reconfigure/server.h"
#include "hanse_sonarlocalization/ParticleFilterConfig.h"
#include "particle_filter.h"

class Localization {
public:
    Localization(ros::NodeHandle handle);

private:
    hanse_sonarlocalization::ParticleFilterConfig config;
    dynamic_reconfigure::Server<hanse_sonarlocalization::ParticleFilterConfig> reconfigServer;


    struct ParamHelper {
	ParticleFilter::Params params;
	ParamHelper();
    };
    ParamHelper paramHelper;

    ParticleFilter particleFilter;

    ros::NodeHandle nh;

    ros::Publisher particlePublisher;
    ros::Publisher positionPublisher;
    ros::Subscriber sonarSubscriber;
    ros::Subscriber positionSubscriber;
    ros::Subscriber imuSubscriber;

    ros::Time lastMsgTime;

    std::deque<sensor_msgs::Imu> imuQueue;

    geometry_msgs::Pose poseFrom2DPosition(Eigen::Affine2f position, float z = 0.0);

    void reconfigure(hanse_sonarlocalization::ParticleFilterConfig &newConfig, uint32_t level);
    void positionCallback(const geometry_msgs::PoseWithCovarianceStamped &pose);
    void sonarCallback(const hanse_msgs::WallDetection &msg);
    void imuCallback(const sensor_msgs::Imu &msg);
};

#endif
