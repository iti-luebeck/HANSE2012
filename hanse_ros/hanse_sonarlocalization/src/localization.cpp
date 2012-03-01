#include <Eigen/Geometry>
#include "localization.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"

Localization::Localization(ros::NodeHandle handle) :
    particleFilter(config),
    nh(handle),
    particlePublisher(handle.advertise<geometry_msgs::PoseArray>("/hanse/particle_markers", 1)),
    positionPublisher(handle.advertise<geometry_msgs::PoseStamped>("/hanse/estimated_position", 1)),
    sonarSubscriber(handle.subscribe("/hanse/sonar/laser_scan", 1, &Localization::sonarCallback, this))
{
    reconfigServer.setCallback(boost::bind(&Localization::reconfigure, this, _1, _2));
    particleFilter.resetPosition();
}

void Localization::reconfigure(hanse_sonarlocalization::ParticleFilterConfig &newConfig, uint32_t level)
{
    particleFilter.reconfigure(newConfig);
}

void Localization::sonarCallback(const sensor_msgs::LaserScan &msg)
{
    particleFilter.perturb();
    particleFilter.weightParticles(msg);
    particleFilter.resample();

    geometry_msgs::PoseStamped position;
    position.header.frame_id = "/map";
    position.pose = poseFrom2DPosition(particleFilter.estimatedPosition());

    positionPublisher.publish(position);

    geometry_msgs::PoseArray particles;

    particles.header.frame_id = "/map";

    for (auto &particle : particleFilter.getParticles()) {
	particles.poses.push_back(poseFrom2DPosition(particle.position));
    }
    particlePublisher.publish(particles);
}

geometry_msgs::Pose Localization::poseFrom2DPosition(Eigen::Affine2f position2D)
{
    geometry_msgs::Pose pose;

    Eigen::Vector2f position = position2D.translation();
    pose.position.x = position(0);
    pose.position.y = position(1);
    pose.position.z = 0.0;
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
