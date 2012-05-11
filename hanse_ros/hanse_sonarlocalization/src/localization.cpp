#include <Eigen/Geometry>
#include <Eigen/Core>
#include "util.h"
#include "localization.h"
#include "geometry_msgs/PoseArray.h"

ros::Time p_start, p_end,
    p_weight_start, p_weight_end,
    p_resample_start, p_resample_end,
    p_perturb_start, p_perturb_end,
    p_move_start, p_move_end
    ;

Localization::ParamHelper::ParamHelper()
{
    bool ok = true;
    ok &= ros::param::get("~map_image", params.map_image);
    ok &= ros::param::get("~map_pixel_size", params.map_pixel_size);
    ok &= ros::param::get("~map_threshold", params.map_threshold);
    if (!ok) {
	ROS_ERROR("Missing parameters");
    }
}

Localization::Localization(ros::NodeHandle handle) :
    particleFilter(config, paramHelper.params),
    nh(handle),
    particlePublisher(handle.advertise<geometry_msgs::PoseArray>("localization/viz/particles", 1)),
    positionPublisher(handle.advertise<geometry_msgs::PoseStamped>("position/estimate", 1)),
    mapPublisher(handle.advertise<nav_msgs::OccupancyGrid>("localization/viz/map", 1)),
    sonarSubscriber(handle.subscribe("sonar/scan/walls", 1, &Localization::sonarCallback, this)),
    positionSubscriber(handle.subscribe("/initialpose", 1, &Localization::positionCallback, this)),
    imuSubscriber(handle.subscribe("imu", 10, &Localization::imuCallback, this))
{
    reconfigServer.setCallback(boost::bind(&Localization::reconfigure, this, _1, _2));
    particleFilter.resetPosition();
}

void Localization::reconfigure(hanse_sonarlocalization::ParticleFilterConfig &newConfig, uint32_t level)
{
    particleFilter.reconfigure(newConfig);
}

void Localization::positionCallback(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    particleFilter.setPosition(localization::positionFromPose(pose.pose.pose));
}

void Localization::sonarCallback(const hanse_msgs::WallDetection &msg)
{
    p_start = ros::Time::now();
    bool update = false;
    while ((!imuQueue.empty()) && imuQueue.front().header.stamp < msg.header.stamp) {
	particleFilter.addImuMessage(imuQueue.front());
	imuQueue.pop_front();
	update = true;
    }

    

    p_perturb_start = ros::Time::now();
    particleFilter.perturb();
    p_perturb_end = ros::Time::now();
    p_move_start = ros::Time::now();
    if (!lastMsgTime.isZero()) {
	ros::Duration timePassed = msg.header.stamp - lastMsgTime;
	particleFilter.move(timePassed.toSec());
    }
    p_move_end = ros::Time::now();
    lastMsgTime = msg.header.stamp;

    if (update) {
	particleFilter.imuUpdate();
    }


    p_weight_start = ros::Time::now();
    particleFilter.weightParticles(msg);
    p_weight_end = ros::Time::now();
    p_resample_start = ros::Time::now();
    particleFilter.resample();
    p_resample_end = ros::Time::now();

    geometry_msgs::PoseStamped position;
    position.header.frame_id = "/map";
    position.pose = poseFrom2DPosition(particleFilter.estimatedPosition(), 1.0);

    positionPublisher.publish(position);

    geometry_msgs::PoseArray particles;

    particles.header.frame_id = "/map";

    int count = 0;
    for (auto &particle : particleFilter.getParticles()) {
	particles.poses.push_back(poseFrom2DPosition(particle.position));
	count++;
	if (count > 1000)
	    break;
    }
    particlePublisher.publish(particles);
    p_end = ros::Time::now();
    /*
    ROS_INFO("Duration %f (w: %f, r: %f, p: %f, m: %f)",
	     (p_end-p_start).toSec(),
	     (p_weight_end-p_weight_start).toSec(),
	     (p_resample_end-p_resample_start).toSec(),
	     (p_perturb_end-p_perturb_start).toSec(),
	     (p_move_end-p_move_start).toSec()
	     );
    */

    ros::Time now = ros::Time::now();

    if (lastMapTime == ros::Time() || (now - lastMapTime).toSec() > 5) {
	mapPublisher.publish(particleFilter.map().occupancyGrid());
	lastMapTime = now;
    }
}

void Localization::imuCallback(const sensor_msgs::Imu &msg)
{
    if (!imuQueue.empty() && imuQueue.front().header.stamp > msg.header.stamp)
	imuQueue.clear();
    imuQueue.push_back(msg);
}

geometry_msgs::Pose Localization::poseFrom2DPosition(Eigen::Affine2f position2D, float z)
{
    geometry_msgs::Pose pose;

    Eigen::Vector2f position = position2D.translation();
    pose.position.x = position(0);
    pose.position.y = position(1);
    pose.position.z = z;
    Eigen::Rotation2D<float> rotation(0);
    rotation.fromRotationMatrix(position2D.rotation());
    Eigen::Quaternion<float> rotation3D(Eigen::AngleAxis<float>(rotation.angle(), Eigen::Vector3f(0,0,1)));
    pose.orientation.x = rotation3D.x();
    pose.orientation.y = rotation3D.y();
    pose.orientation.z = rotation3D.z();
    pose.orientation.w = rotation3D.w();
    return pose;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    Localization localization(n);

    ros::spin();

    return 0;
}
