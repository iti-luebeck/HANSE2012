#include <algorithm>
#include "particle_filter.h"

ParticleFilter::ParticleFilter(hanse_sonarlocalization::ParticleFilterConfig config, Params params) :
    config(config),
    worldMap(params.map_image, params.map_pixel_size, params.map_threshold)
{
}

void ParticleFilter::reconfigure(hanse_sonarlocalization::ParticleFilterConfig config)
{
    this->config = config;
    if (particles.size() != (unsigned)config.particle_count) {
	resetPosition();
    }
}

Eigen::ArrayXXf ParticleFilter::randNormal(int m, int n, double sigma, double mu) {
    Eigen::ArrayXXf X(m,n);
    Eigen::ArrayXXf U1 = (Eigen::ArrayXXf::Random(m, n) + 1.0000001192092896f) / 2;
    Eigen::ArrayXXf U2 = (Eigen::ArrayXXf::Random(m, n) + 1.0000001192092896f) / 2;

    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            double r = sqrt(-2 * log(U1(i,j)) / log(2));
            assert(!isinf(r));
            double phi = 2 * M_PI * U2(i,j);
            r = sigma * r * cos(phi) + mu;
            X(i,j) = r;
        }
    }
    return X;
}

void ParticleFilter::resetPosition()
{
    Eigen::Array2f mapSize = worldMap.mapSize();
    Eigen::ArrayXXf randomPositions = (Eigen::ArrayXXf::Random(2, config.particle_count) + 1) / 2;
    Eigen::ArrayXXf randomRotations = Eigen::ArrayXXf::Random(1, config.particle_count) * M_PI;

    particles.clear();

    for (int i = 0; i < config.particle_count; i++) {
	Particle particle;
	particle.weight = 1.0;
	particle.position = Eigen::Translation<float, 2>(randomPositions.col(i) * mapSize) * Eigen::Rotation2D<float>(randomRotations(0, i));
	particles.push_back(particle);
    }
}

void ParticleFilter::setPosition(Eigen::Affine2f position)
{
    Particle p;
    p.weight = 1.0;
    p.position = position;
    particles.clear();
    particles.resize(config.particle_count, p);
}

void ParticleFilter::perturb()
{
    Eigen::MatrixXf distance = randNormal(2, config.particle_count, config.perturb_position, 0);
    Eigen::MatrixXf rotation = randNormal(1, config.particle_count, config.perturb_rotation, 0);
    for (int i = 0; i < config.particle_count; i++) {
	Eigen::Affine2f position = particles[i].position;
	position = Eigen::Translation<float, 2>(distance.col(i)) * position * Eigen::Rotation2D<float>(rotation(0, i));
	particles[i].position = position;
    }
}

void ParticleFilter::weightParticles(sensor_msgs::LaserScan const &laserScan)
{
    bestParticle.weight = -1;
    for (auto &particle : particles) {
	weightParticle(particle, laserScan);
    }
    ROS_INFO("best particle weight %f", bestParticle.weight);
}

void ParticleFilter::resample()
{
    float sum = 0;
    for (auto &particle : particles) {
	sum += particle.weight;
    }
    if (sum == 0 || isnan(sum)) {
	ROS_INFO("zero or NaN weights: %f", sum);
	return;
    }
    float eff = 0;
    for (auto &particle : particles) {
	eff += (particle.weight * particle.weight) / (sum * sum);
    }

    if (2 * eff >= config.particle_count) {
	ROS_INFO("efficient enough");
	return;
    }

    std::vector<float> commulativeWeights;
    commulativeWeights.reserve(config.particle_count);

    float last = 0;
    for (auto &particle : particles) {
	last = particle.weight / sum + last;
	commulativeWeights.push_back(last);
    }

    Eigen::ArrayXXf rand = (Eigen::ArrayXXf::Random(1, config.particle_count) + 1) / 2;

    ParticleVector newParticles;
    newParticles.reserve(config.particle_count);

    for (int i = 0; i < config.particle_count; i++) {
	auto iterator = std::lower_bound(commulativeWeights.begin(), commulativeWeights.end(), rand(0, i));
	int position = iterator - commulativeWeights.begin();
	newParticles.push_back(particles[position]);
    }

    particles = std::move(newParticles);
}

Eigen::Affine2f ParticleFilter::estimatedPosition()
{
    Eigen::Vector2f meanPosition(0, 0);
    Eigen::Vector2f meanDirection(0, 0);
    for (auto &particle : particles) {
	meanPosition += particle.position.translation();
	meanDirection += particle.position.rotation() * Eigen::Vector2f(1, 0);
    }
    meanPosition /= particles.size();
    double meanAngle = atan2f(meanDirection.y(), meanDirection.x());

    return Eigen::Translation2f(meanPosition) * Eigen::Rotation2D<float>(meanAngle);
}

void ParticleFilter::weightParticle(Particle &particle, sensor_msgs::LaserScan const &laserScan)
{
    float sigma = config.weight_sigma;
    float weight = 1;
    float auvDistance = worldMap.wallDistance(particle.position.translation());

    if (auvDistance < 0) {
	weight = 0;
    } else {
	for (unsigned int i = 0; i < laserScan.ranges.size(); i++) {
	    if (laserScan.ranges[i] < laserScan.range_min || laserScan.ranges[i] > laserScan.range_max)
		continue;
	    float angle = laserScan.angle_min + i * laserScan.angle_increment;
	    Eigen::Rotation2D<float> rotation(angle);

	    float range = laserScan.ranges[i];

	    Eigen::Translation<float, 2> translation(1, 0);
	    Eigen::Affine2f wallPosition = particle.position.rotation() * rotation * translation;

	    // expected wall distance
	    float expectedDistance = worldMap.directedWallDistance(particle.position.translation(), wallPosition.translation(), laserScan.range_max);

	    float distance = fabsf(expectedDistance - range);

	    distance -= config.bend_distance;
	    if (distance < 0)
		distance /= config.bend_factor;
	    distance += config.bend_distance / config.bend_factor;

	    weight *= expf( - 0.5 * powf(distance / sigma, 2));
	}
    }
    particle.weight = weight;
    if (particle.weight > bestParticle.weight) {
	bestParticle = particle;
    }
}
