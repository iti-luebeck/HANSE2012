#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <vector>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include "hanse_sonarlocalization/ParticleFilterConfig.h"
#include "particle.h"
#include "world_map.h"

class ParticleFilter {
public:
    typedef std::vector<Particle, Eigen::aligned_allocator<Particle> > ParticleVector;

    ParticleFilter(hanse_sonarlocalization::ParticleFilterConfig config);


    void reconfigure(hanse_sonarlocalization::ParticleFilterConfig config);

    void resetPosition();
    void setPosition(Eigen::Affine2f position);
    void perturb();
    void weightParticles(sensor_msgs::LaserScan const &laserScan);
    void resample();
    Eigen::Affine2f estimatedPosition();

    const ParticleVector &getParticles() const { return particles; }
private:
    void weightParticle(Particle &particle, sensor_msgs::LaserScan const &laserScan);;

    Particle bestParticle;

    Eigen::ArrayXXf randNormal(int m, int n, double sigma, double mu);

    hanse_sonarlocalization::ParticleFilterConfig config;
    ParticleVector particles;
    WorldMap worldMap;
};

#endif
