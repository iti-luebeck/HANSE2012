#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <Eigen/Geometry>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
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

    geometry_msgs::Pose poseFrom2DPosition(Eigen::Affine2f position);

    void reconfigure(hanse_sonarlocalization::ParticleFilterConfig &newConfig, uint32_t level);
    void sonarCallback(const sensor_msgs::LaserScan &msg);
};

#endif
