#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <vector>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include "hanse_sonarlocalization/ParticleFilterConfig.h"
#include "particle.h"
#include "world_map.h"

class ParticleFilter {
public:
    typedef std::vector<Particle, Eigen::aligned_allocator<Particle> > ParticleVector;
    struct Params {
	std::string map_image;
	double map_pixel_size;
	double map_threshold;
    };

    ParticleFilter(hanse_sonarlocalization::ParticleFilterConfig config, Params params);


    void reconfigure(hanse_sonarlocalization::ParticleFilterConfig config);

    void resetPosition();
    void setPosition(Eigen::Affine2f position);
    void move(float seconds);
    void perturb();
    void weightParticles(sensor_msgs::LaserScan const &laserScan);
    void resample();
    void addImuMessage(sensor_msgs::Imu const &imu);
    void imuUpdate();
    Eigen::Affine2f estimatedPosition();

    const ParticleVector &getParticles() const { return particles; }
private:
    void weightParticle(Particle &particle, sensor_msgs::LaserScan const &laserScan);

    Particle bestParticle;

    Eigen::ArrayXXf randNormal(int m, int n, double sigma, double mu);

    hanse_sonarlocalization::ParticleFilterConfig config;
    ParticleVector particles;
    WorldMap worldMap;

    // these are in particle coordinates (unlike the velocity of a particle which is in world coordinates)
    Eigen::Quaternionf lastImuOrientation; // coordinates of last particle update
    Eigen::Affine3f imuPosition;
    Eigen::Vector3f imuVelocity;

    ros::Time lastImuMsgTime;
};

#endif