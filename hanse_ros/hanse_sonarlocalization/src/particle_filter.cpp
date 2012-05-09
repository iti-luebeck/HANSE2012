#include <algorithm>
#include "util.h"
#include "particle_filter.h"
#include <trng/mt19937.hpp>
#include <trng/normal_dist.hpp>

ParticleFilter::ParticleFilter(hanse_sonarlocalization::ParticleFilterConfig config, Params params) :
    config(config),
    worldMap(params.map_image, params.map_pixel_size, params.map_threshold)
{
    lastImuOrientation = Eigen::Quaternion<float>::Identity();
    imuPosition = Eigen::Affine3f::Identity();
    imuVelocity = Eigen::Vector3f(0, 0, 0);
}

void ParticleFilter::reconfigure(hanse_sonarlocalization::ParticleFilterConfig config)
{
    this->config = config;
    if (particles.size() != (unsigned)config.particle_count) {
	resetPosition();
    }
}

Eigen::ArrayXXf ParticleFilter::randNormal(int m, int n, float sigma, float mu) {
    static trng::mt19937 R;
    trng::normal_dist<float> normal(mu, sigma);

    Eigen::ArrayXXf X(m,n);
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
	    X(i, j) = normal(R);
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
	particle.velocity = Eigen::Vector2f(0, 0);
	particle.position = Eigen::Translation<float, 2>(randomPositions.col(i) * mapSize) * Eigen::Rotation2D<float>(randomRotations(0, i));
	particles.push_back(particle);
    }
}

void ParticleFilter::setPosition(Eigen::Affine2f position)
{
    Particle p;
    p.weight = 1.0;
    p.position = position;
    p.velocity = Eigen::Vector2f(0, 0);
    particles.clear();
    particles.resize(config.particle_count, p);
}

void ParticleFilter::move(float seconds)
{
    for (auto &particle : particles) {
	particle.position.pretranslate(particle.velocity * seconds);
    }
}

void ParticleFilter::perturb()
{
    Eigen::MatrixXf distance = randNormal(2, config.particle_count, config.perturb_position, 0);
    Eigen::MatrixXf acceleration = randNormal(2, config.particle_count, config.perturb_velocity, 0);
    Eigen::MatrixXf rotation = randNormal(1, config.particle_count, config.perturb_rotation, 0);
    for (int i = 0; i < config.particle_count; i++) {
	particles[i].velocity = (particles[i].velocity * 0.99) + Eigen::Vector2f(acceleration.col(i));
	particles[i].position.pretranslate(Eigen::Vector2f(distance.col(i)));
	particles[i].position.rotate(rotation(0, i));
    }
}

void ParticleFilter::weightParticles(const hanse_msgs::WallDetection &msg)
{
    bestParticle.weight = -1;
    for (auto &particle : particles) {
	weightParticle(particle, msg);
    }
    //ROS_INFO("best particle weight %f", bestParticle.weight);
}

void ParticleFilter::resample()
{
    int particle_count = particles.size();

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

    if (2 * eff >= particle_count) {
	ROS_INFO("efficient enough");
	return;
    }

    std::vector<float> commulativeWeights;
    commulativeWeights.reserve(particle_count);

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
	if (position >= particle_count)
	    position = particle_count - 1;
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

void ParticleFilter::weightParticle(Particle &particle, const hanse_msgs::WallDetection &msg)
{
    float sigma = config.weight_sigma;
    float weight = 1;
    float auvDistance = worldMap.wallDistance(particle.position.translation());
    float weight_scaling = config.weight_scaling;

    if (auvDistance < 0) {
	weight = 1e-10;
    } else if (msg.wallDetected && !msg.distances.empty()) {
	Eigen::Rotation2D<float> rotation(msg.headPosition);

	Eigen::Translation<float, 2> translation(1, 0);
	Eigen::Affine2f wallPosition = particle.position.rotation() * rotation * translation;

	// expected wall distance
	float expectedDistance =
	    worldMap.directedWallDistance(particle.position.translation(),
					  wallPosition.translation(),
					  2 * msg.range);

	float distance = HUGE_VAL;
	for (float range : msg.distances)
	    distance = std::min(distance, fabsf(expectedDistance - range));

	distance -= config.bend_distance;
	if (distance < 0)
	    distance /= config.bend_factor;
	distance += config.bend_distance / config.bend_factor;

	weight *= weight_scaling + (1 - weight_scaling) * expf( - 0.5 * powf(distance / sigma, 2));
    }
    particle.weight = weight;
    if (particle.weight > bestParticle.weight) {
	bestParticle = particle;
    }
}

void ParticleFilter::addImuMessage(sensor_msgs::Imu const &imu)
{
    // TODO: all the acceleration based code needs
    // some rework with real world test data
    Eigen::Vector3f acceleration(imu.linear_acceleration.x,
				 imu.linear_acceleration.y,
				 imu.linear_acceleration.z);

    // WARNING: yes the parameter order for eigen is the much saner
    // w x y z _BUT_ the xsens driver writers were too stupid to read the
    // xsens docs which clearly state that it also uses the w x y z order and
    // just assumed it would be x y z w, passing in the incorrect data, 
    Eigen::Quaternionf orientation(imu.orientation.x,
				   imu.orientation.y,
				   imu.orientation.z,
				   imu.orientation.w);

    orientation = Eigen::AngleAxis<float>(M_PI, Eigen::Vector3f(0, 1, 0)) * orientation;

    Eigen::Matrix3f rotation = orientation.toRotationMatrix();

    Eigen::AngleAxis<float> tmp;
    tmp.fromRotationMatrix(rotation);
    
    ROS_INFO("%f %f %f [%f]", tmp.axis().x(), tmp.axis().y(), tmp.axis().z(), tmp.angle());


    float interval = (imu.header.stamp - lastImuMsgTime).toSec();

    Eigen::Vector3f worldAcceleration = orientation * acceleration;
    worldAcceleration.z() = 0;
    float absoluteAcceleration = worldAcceleration.norm();
    if (absoluteAcceleration > 0.1) {
      worldAcceleration *= 0.9*(absoluteAcceleration-0.1) / absoluteAcceleration;
      imuVelocity +=  interval * (orientation * acceleration);
    }
    imuPosition = Eigen::Translation3f(imuPosition.translation() + imuVelocity * interval) * orientation;

    lastImuMsgTime = imu.header.stamp;
}

void ParticleFilter::imuUpdate()
{
    // TODO: handle initialization
    Eigen::Affine3f imuRelativePosition = lastImuOrientation.inverse() * imuPosition;
    Eigen::Affine2f relativePosition2d = localization::positionFromAffine3(imuRelativePosition);
    Eigen::Vector2f relativeVelocity2d(imuVelocity.x(), imuVelocity.y());
    Eigen::Rotation2D<float> rot(0);
    rot.fromRotationMatrix(relativePosition2d.rotation());
    //ROS_INFO("FOO dx=%f dy=%f dtheta=%f", relativePosition2d.translation().x(), relativePosition2d.translation().y(), rot.angle());
    //ROS_INFO("FOO ddx=%f ddy=%f", relativeVelocity2d.x(), relativeVelocity2d.y());

    if (config.imu_motion) {
	for (auto &particle : particles) {
	    particle.position = particle.position * relativePosition2d;
	    particle.velocity += relativeVelocity2d; // TODO handle rotation offset
	}
    } else {
	for (auto &particle : particles) {
	    particle.position = particle.position * rot;
	}
    }
    imuVelocity = Eigen::Vector3f(0, 0, 0);
    lastImuOrientation = imuPosition.rotation();
    imuPosition = imuRelativePosition.rotation();
}
