#include <Eigen/Geometry>
#include <Eigen/Core>
#include "util.h"
#include "localization.h"
#include "geometry_msgs/PoseArray.h"

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
    particlePublisher(handle.advertise<geometry_msgs::PoseArray>("location/particle_markers", 1)),
    positionPublisher(handle.advertise<geometry_msgs::PoseStamped>("location/estimated_position", 1)),
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
    bool update = false;
    while ((!imuQueue.empty()) && imuQueue.front().header.stamp < msg.header.stamp) {
	particleFilter.addImuMessage(imuQueue.front());
	imuQueue.pop_front();
	update = true;
    }
    if (update) {
	particleFilter.imuUpdate();
    }

    particleFilter.perturb();
    if (!lastMsgTime.isZero()) {
	ros::Duration timePassed = msg.header.stamp - lastMsgTime;
	particleFilter.move(timePassed.toSec());
    }
    lastMsgTime = msg.header.stamp;
    particleFilter.weightParticles(msg);
    particleFilter.resample();

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
}

void Localization::imuCallback(const sensor_msgs::Imu &msg)
{
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
